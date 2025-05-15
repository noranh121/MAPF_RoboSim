from flask import Flask, render_template, request, redirect, url_for, flash, jsonify, send_file
import subprocess
import os
import time
import signal
import io


app = Flask(__name__)
app.secret_key = "secret-key"
UPLOAD_FOLDER = 'uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

# Absolute path to the algorithms folder
MAPF_ros2_ws = os.getcwd()
ALGO_FOLDER = os.path.join(
    MAPF_ros2_ws,
    'src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/algorithms'
)
MAP_FOLDER = os.path.join(
    MAPF_ros2_ws,
    'src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks'
)
SCEN_FOLDER = os.path.join(
    MAPF_ros2_ws,
    'src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/scenarios'
)


# Predefined built-in algorithms
# builtin_algorithms = ["Algorithm A", "Algorithm B", "Algorithm C"]
# builtin_maps = ["map1"]
# builtin_scenarios = ["scen1"]
builtin_maps = []
builtin_scenarios = []
builtin_algorithms = []

# Dynamic list of uploaded algorithms
uploaded_algorithms = []
uploaded_maps = []
uploaded_scenarios = []


def get_builtin_files(folder):
    return [f for f in os.listdir(folder) if os.path.isfile(os.path.join(folder, f))]


@app.route('/home')
def home():
    builtin_algorithms = get_builtin_files(ALGO_FOLDER)
    builtin_maps = get_builtin_files(MAP_FOLDER)
    builtin_scenarios = get_builtin_files(SCEN_FOLDER)
    all_algorithms = builtin_algorithms + uploaded_algorithms
    all_maps = builtin_maps + uploaded_maps
    all_scens = builtin_scenarios + uploaded_scenarios
    return render_template('home.html', algorithms=all_algorithms, maps=all_maps, scenarios=all_scens)


@app.route('/about')
def about():
    return render_template('about.html')

@app.route('/')
def main():
    return render_template('main.html')

@app.route('/info')
def info():
    return render_template('info.html')


@app.route('/upload-algorithm', methods=['POST'])
def upload_algorithm():
    if 'file' not in request.files:
        flash('No file part', "part1")
        return redirect(url_for('home'))

    file = request.files['file']
    if file.filename == '':
        flash('No file selected', "part1")
        return redirect(url_for('home'))

    if file:
        try:
            MAPF_ros2_ws=os.getcwd()
            path = MAPF_ros2_ws + '/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/algorithms/'

            filepath = os.path.join(path, file.filename)
            file.save(filepath)

            if file.filename not in uploaded_algorithms:
                uploaded_algorithms.append(file.filename)

            flash(f'Algorithm "{file.filename}" uploaded successfully!', "part1")
            return redirect(url_for('home'))
        except Exception as e:
            flash(f"An error occurred: {e}", "part1")


@app.route('/upload-benchmark', methods=['POST'])
def upload_benchmark():
    try:
        if 'file' not in request.files:
            flash('No file part', "part1")
            return redirect(url_for('home'))

        file = request.files['file']
        if file.filename == '':
            flash('No file selected', "part1")
            return redirect(url_for('home'))
        
        if not file.filename.endswith('.txt'):
            flash('Only .txt files are allowed', "part1")
            return redirect(url_for('home'))

        if file:
            MAPF_ros2_ws=os.getcwd()
            path = MAPF_ros2_ws + '/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/'

            filepath = os.path.join(path, file.filename)
            file.save(filepath)

            if file.filename not in uploaded_maps:
                uploaded_maps.append(file.filename)

            flash(f'Benchamrk "{file.filename}" uploaded successfully!', "part1")
            return redirect(url_for('home'))
    except Exception as e:
        flash(f"An error occurred: {e}", "part1")

    return redirect(url_for('home'))
    
@app.route('/upload-scenario', methods=['POST'])
def upload_scenario():
    try:
        if 'file' not in request.files:
            flash('No file part', "part1")
            return redirect(url_for('home'))

        file = request.files['file']
        if file.filename == '':
            flash('No file selected', "part1")
            return redirect(url_for('home'))
        
        if not file.filename.endswith('.txt'):
            flash('Only .txt files are allowed', "part1")
            return redirect(url_for('home'))

        if file:
            MAPF_ros2_ws=os.getcwd()
            path = MAPF_ros2_ws + '/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/scenarios/'

            filepath = os.path.join(path, file.filename)
            file.save(filepath)

            if file.filename not in uploaded_scenarios:
                uploaded_scenarios.append(file.filename)

            flash(f'Scenario "{file.filename}" uploaded successfully', "part1")
            return redirect(url_for('home'))

    except Exception as e:
        flash(f"An error occurred: {str(e)}", "part1")

    return redirect(url_for('home'))
    


result = None

def stop_simulation():
    global result
    if result and result.poll() is None:
        os.killpg(os.getpgid(result.pid), signal.SIGINT)
        result.wait()
        result = None

@app.route('/simulate', methods=['POST'])
def simulate():
    global result
    stop_simulation()
    time.sleep(5)

    ros_distro = os.environ.get("ROS_DISTRO")


    selected_algo = request.form.get('algorithm', 'None')
    selected_map = request.form.get('map', 'None')
    selected_scen = request.form.get('scenario', 'None')

    flash(f'Simulation started with algorithm "{selected_algo}", map "{selected_map}", scenario "{selected_scen}, and ROS_DISTRO:={ros_distro}"', 'part2')
    try:
        upfront_command = "wmctrl -a 'Gazebo'"
        
        command = f"ros2 launch a_star_tb3 empty_world.launch.py benchmark:={selected_map} scenario:={selected_scen} algorithm:={selected_algo}"
        commands = [
            "cd ~/MAPF_RoboSim/ros2_ws",
            "colcon build",
            f"source /opt/ros/{ros_distro}/setup.bash",
            "source install/setup.bash",
            "source install/local_setup.bash",
            command
        ]
        

        full_command = " && ".join(commands)

        result = subprocess.Popen(
            full_command,
            shell=True,
            executable="/bin/bash",
            preexec_fn=os.setsid,
            text=True,stderr=subprocess.PIPE)

        time.sleep(5)

        process_upfront = subprocess.Popen(
            upfront_command,
            shell=True,
            executable="/bin/bash",
            text=True,stderr=subprocess.PIPE)

        stdout, stderr = result.communicate()
        process_upfront.communicate()
        if result.returncode == 0:
            flash(f'success output:{stdout}', "part2")
            time.sleep(5)
            return redirect(url_for('home'))
        else:
            print(stdout)
            flash(f"Error occurred: {stderr}","part1")
            time.sleep(5)
            return redirect(url_for('home'))
    except Exception as e:
        flash(f"An error occurred: {e}", "part1")
        time.sleep(5)
        return redirect(url_for('home'))




@app.route('/export', methods=['POST'])
def export():
    content = "RESULTS!!!"
    filename = "simulation_stats.txt"
    # try:

    #     MAPF_ros2_ws=os.getcwd()
    #     path = MAPF_ros2_ws + '/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/'

    #     path = os.path.join(path, "stats.txt")
    #     with open(path, 'r', encoding='utf-8') as f:
    #         data_to_export = f.read()

    #     # Path to the temporary file to store data
    #     export_file_path = os.path.join(app.config['UPLOAD_FOLDER'], "exported_data.txt")

    #     # Write data to the file
    #     with open(export_file_path, 'w', encoding='utf-8') as file:
    #         file.write(data_to_export)

    # except Exception as e:
    #     flash(f"An error occurred: {str(e)}")
    #     return redirect(url_for('home'))
    
    buffer = io.BytesIO()
    buffer.write(content.encode())
    buffer.seek(0)

    #flash(f"Stats Exprted Successfully", "part2")

    return send_file(
        buffer,
        as_attachment=True,
        download_name=filename,
        mimetype='text/plain'
    )
 

if __name__ == '__main__':
    app.run(debug=True)