

import numpy as np
import os
import traceback
from flask import Flask, request, jsonify, send_from_directory
from flask.json.provider import DefaultJSONProvider
from flask_cors import CORS
from ortools.sat.python import cp_model

# --- 1. Fix for Numpy JSON Serialization ---
class NumpyJSONProvider(DefaultJSONProvider):
    def default(self, obj):
        if isinstance(obj, (np.integer, np.floating)):
            return float(obj) if isinstance(obj, np.floating) else int(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super().default(obj)

app = Flask(__name__)
app.json = NumpyJSONProvider(app) 
CORS(app)

class OptimizationLogger:
    def __init__(self):
        self.logs = []
    def log(self, message):
        print(message) 
        self.logs.append(message)

def generate_sub_targets(priority_areas, density, logger):
    sub_targets = set()
    for area in priority_areas:
        center_x, center_y = float(area['center'][0]), float(area['center'][1])
        radius, step = float(area['radius']), float(density)
        
        x_range = np.arange(center_x - radius, center_x + radius + step, step)
        y_range = np.arange(center_y - radius, center_y + radius + step, step)
        
        for x in x_range:
            for y in y_range:
                xf, yf = float(x), float(y)
                if np.sqrt((xf - center_x)**2 + (yf - center_y)**2) <= radius:
                    sub_targets.add((round(xf, 2), round(yf, 2)))
    
    if not sub_targets:
         logger.log("Warning: Could not generate sub-targets. Check density/radius.")
         return []
    return list(sub_targets)

# --- UPGRADED CORE ALGORITHM ---
def solve_deployment(params):
    logger = OptimizationLogger()
    logger.log("--- INITIALIZING HETEROGENEOUS MISSION ---")
    
    sensor_types = params.get('sensor_types', [])
    if not sensor_types:
        return {"error": "No sensor inventory provided.", "logs": logger.logs}

    # 1. Generate Targets & Grid
    logger.log(f"Mapping targets (Density={params['density']})...")
    sub_targets = generate_sub_targets(params['priority_areas'], params['density'], logger)
    locations = []
    grid_step = 1.0 
    for x in np.arange(0, params['width'] + grid_step/2, grid_step):
        for y in np.arange(0, params['height'] + grid_step/2, grid_step):
            locations.append((float(x), float(y)))

    num_locations, num_targets = len(locations), len(sub_targets)
    logger.log(f"Grid: {num_locations} sites | Targets: {num_targets} | Sensor Types: {len(sensor_types)}")

    # 2. Pre-calculate 3D Coverage Matrix covers[(location_idx, target_idx, sensor_type_idx)]
    logger.log("Calculating Multi-Dimensional Line-of-Sight matrix...")
    covers = {}
    for t, sensor in enumerate(sensor_types):
        range_sq = float(sensor['range']) ** 2 
        for i in range(num_locations):
            lx, ly = locations[i]
            for j in range(num_targets):
                tx, ty = sub_targets[j]
                if (lx - tx)**2 + (ly - ty)**2 <= range_sq:
                    covers[(i, j, t)] = 1
                
    # 3. ILP Model
    logger.log("Building Constraint Programming Model...")
    model = cp_model.CpModel()
    
    # x[(i, t)] = 1 if sensor type 't' is placed at location 'i'
    x = {}
    for i in range(num_locations):
        for t in range(len(sensor_types)):
            x[(i, t)] = model.NewBoolVar(f'x_loc{i}_type{t}')
            
    # Max ONE sensor per physical location (can't stack a radar and camera on the exact same pole)
    for i in range(num_locations):
        model.AddAtMostOne([x[(i, t)] for t in range(len(sensor_types))])

    y = [model.NewBoolVar(f'y_{j}') for j in range(num_targets)]   
    
    max_total_sensors = sum(int(s['max_qty']) for s in sensor_types)
    num_covers = [model.NewIntVar(0, max_total_sensors, f'nc_{j}') for j in range(num_targets)]

    # Channeling: Link physical placement to target coverage
    for j in range(num_targets):
        sensors_covering_j = [x[(i, t)] for i in range(num_locations) for t in range(len(sensor_types)) if covers.get((i, j, t), 0) == 1]
        if sensors_covering_j:
            model.Add(cp_model.LinearExpr.Sum(sensors_covering_j) == num_covers[j])
        else:
            model.Add(num_covers[j] == 0)

        model.Add(num_covers[j] > 0).OnlyEnforceIf(y[j])
        model.Add(num_covers[j] == 0).OnlyEnforceIf(y[j].Not())

    # Inventory Constraints: Limit based on specific sensor types
    for t, sensor in enumerate(sensor_types):
        model.Add(cp_model.LinearExpr.Sum([x[(i, t)] for i in range(num_locations)]) <= int(sensor['max_qty']))

    # Objective Function
    total_obj = []
    
    # (+) Reward for covering a target
    for j in range(num_targets):
        total_obj.append(y[j] * params['w_coverage'])
        
    # (-) Penalty automatically calculated based on the sensor's range
    # This ensures large sensors are inherently "costlier" to use than small sensors.
    for i in range(num_locations):
        for t, sensor in enumerate(sensor_types):
            auto_cost = int(float(sensor['range']) * 2) 
            total_obj.append(x[(i, t)] * -auto_cost)
        
    # (-) Penalty for overlap to encourage spreading out
    for j in range(num_targets):
        actual_overlap = model.NewIntVar(0, max_total_sensors, f'o_{j}')
        model.Add(actual_overlap == num_covers[j] - 1).OnlyEnforceIf(y[j])
        model.Add(actual_overlap == 0).OnlyEnforceIf(y[j].Not())
        total_obj.append(actual_overlap * -params['w_overlap'])

    model.Maximize(cp_model.LinearExpr.Sum(total_obj))

    # 4. Solve
    logger.log("Engaging Solver...")
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 30.0
    status = solver.Solve(model)

    # 5. Process Results
    if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
        status_name = solver.StatusName(status)
        
        placed = []
        for i in range(num_locations):
            for t, sensor in enumerate(sensor_types):
                if solver.Value(x[(i, t)]) == 1:
                    placed.append({
                        "x": locations[i][0],
                        "y": locations[i][1],
                        "type": sensor['name'],
                        "range": sensor['range'],
                        "color": sensor['color']
                    })
                    
        covered = [sub_targets[j] for j in range(num_targets) if solver.Value(y[j]) == 1]
        uncovered = [sub_targets[j] for j in range(num_targets) if solver.Value(y[j]) == 0]
        
        logger.log(f"DEPLOYMENT SUMMARY: {len(placed)} total sensors placed.")
        
        return {
            "status": status_name,
            "placedSensors": placed,
            "coveredSubTargets": covered,
            "uncoveredSubTargets": uncovered,
            "summary": f"Deployed {len(placed)} sensors to cover {len(covered)} targets.",
            "logs": logger.logs
        }
    else:
        logger.log(f"Optimization Failed: {solver.StatusName(status)}")
        return {"error": "No solution found.", "logs": logger.logs}

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

@app.route('/api/optimize', methods=['POST'])
def optimize_api():
    try:
        data = request.json
        params = {
            'width': float(data.get('width', 50)),
            'height': float(data.get('height', 50)),
            'priority_areas': data.get('priority_areas', []),
            'sensor_types': data.get('sensor_types', []), # Cost is no longer required from the frontend!
            'density': float(data.get('density', 2.0)),
            'w_coverage': 800, 
            'w_overlap': 150, 
        }
        
        results = solve_deployment(params)
        
        if "error" in results:
            return jsonify(results), 400
        return jsonify(results)
        
    except Exception as e:
        print("--- SERVER ERROR ---")
        traceback.print_exc() 
        return jsonify({"error": str(e), "logs": ["Critical Server Error. Check Terminal."]}), 500

if __name__ == '__main__':
    app.run(debug=True, port=5000)