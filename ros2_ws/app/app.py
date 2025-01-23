from flask import Flask, render_template, request, redirect, url_for, flash,jsonify
import subprocess
import os
import time


app = Flask(__name__)
app.secret_key = "secret-key"
UPLOAD_FOLDER = 'uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

# Predefined built-in algorithms
builtin_algorithms = ["Algorithm A", "Algorithm B", "Algorithm C"]
builtin_maps = ["map1"]

# Dynamic list of uploaded algorithms
uploaded_algorithms = []
uploaded_maps = []


@app.route('/')
def dashboard():
    all_algorithms = builtin_algorithms + uploaded_algorithms
    all_maps = builtin_maps + uploaded_maps
    return render_template('index.html', algorithms=all_algorithms, maps=all_maps)


@app.route('/about')
def about():
    return render_template('about.html')


@app.route('/upload', methods=['POST'])
def upload():
    if 'file' not in request.files:
        flash('No file part')
        return redirect(url_for('dashboard'))

    file = request.files['file']
    if file.filename == '':
        flash('No file selected')
        return redirect(url_for('dashboard'))

    if file:
        filepath = os.path.join(app.config['UPLOAD_FOLDER'], file.filename)
        file.save(filepath)

        if file.filename not in uploaded_algorithms:
            uploaded_algorithms.append(file.filename)

        flash(f'Algorithm "{file.filename}" uploaded successfully!')
        return redirect(url_for('dashboard'))


@app.route('/upload-map', methods=['POST'])
def upload_map():
    if 'file' not in request.files:
        flash('No file part')
        return redirect(url_for('dashboard'))

    file = request.files['file']
    if file.filename == '':
        flash('No file selected')
        return redirect(url_for('dashboard'))

    if file:
        MAPF_ros2_ws=os.getcwd()
        parent_directory = os.path.dirname(MAPF_ros2_ws)
        path = parent_directory + '/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/'

        filepath = os.path.join(path, file.filename)
        # file.save(filepath)
        file.save(filepath)

        if file.filename not in uploaded_maps:
            uploaded_maps.append(file.filename)

        flash(f'Map "{file.filename}" uploaded successfully!')
        return redirect(url_for('dashboard'))
    

@app.route('/upload-points', methods=['POST'])
def upload_points():
    flash("file uploaded successfully!")
    return redirect(url_for('dashboard'))
    

@app.route('/simulate', methods=['POST'])
def simulate():
    selected_algo = request.form.get('algorithm', 'None')
    selected_map = request.form.get('map', 'None')
    number = request.form.get('agents' ,'None')
    start_points = request.form.get('start' ,'None')
    end_points = request.form.get('end' ,'None')

    flash(f'Simulation started for "{selected_algo}" on map: "{selected_map}" number of agents "{number}" start points "{start_points}" end points "{end_points}" ')


    try:
        # Get the benchmark file path from the request payload
        #benchmark_file = request.json.get('benchmark', '<default_path>')
        upfront_command = "wmctrl -a 'Gazebo'"
        
        # Define the commands with dynamic benchmark file path
        #command = "ros2 launch a_star_tb3 empty_world.launch.py goal_x:=5 goal_y:=0 start_x:=0 start_y:=0.25 RPM1:=40 RPM2:=20 clearance:=100"
        command = f"ros2 launch a_star_tb3 empty_world.launch.py benchmark:={selected_map} ros2_distro:=galactic"
        commands = [
            "cd ~/MAPF_RoboSim/ros2_ws",
            "colcon build",
            "source install/setup.bash",
            "source install/local_setup.bash",
            command,
            upfront_command
        ]
        

        full_command = " && ".join(commands)

        #result = subprocess.run(full_command, shell=True, executable="/bin/bash", text=True, capture_output=True)
        result = subprocess.Popen(full_command, shell=True, executable="/bin/bash", text=True)

        time.sleep(5)

        process_upfront = subprocess.Popen(upfront_command, shell=True, executable="/bin/bash", text=True)

        result.wait()

        process_upfront.wait()

        if result.returncode == 0:
            return jsonify({"status": "success", "output": result.stdout})
        else:
            return jsonify({"status": "error", "error": result.stderr}), 500
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 500
    # return redirect(url_for('dashboard'))




# @app.route('/algorithm-upload', methods=['GET'])
# def algorithm_upload():
#     return render_template('algorithm_upload.html')

# @app.route('/map-selection', methods=['GET'])
# def map_selection():
#     return render_template('map_selection.html')

@app.route('/home', methods=['GET'])
def home():
    return render_template('home.html')

if __name__ == '__main__':
    app.run(debug=True)
