from flask import Flask, render_template, request, redirect, url_for, flash
import os

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
        filepath = os.path.join(app.config['UPLOAD_FOLDER'], file.filename)
        file.save(filepath)

        if file.filename not in uploaded_maps:
            uploaded_maps.append(file.filename)

        flash(f'Map "{file.filename}" uploaded successfully!')
        return redirect(url_for('dashboard'))


@app.route('/simulate', methods=['POST'])
def simulate():
    selected_algo = request.form.get('algorithm', 'None')
    selected_map = request.form.get('map', 'None')
    number = request.form.get('agents' ,'None')
    start_points = request.form.get('start' ,'None')
    end_points = request.form.get('end' ,'None')
    flash(f'Simulation started for "{selected_algo}" on map: "{selected_map}" number of agents "{number}" start points "{start_points}" end points "{end_points}" ')
    return redirect(url_for('dashboard'))

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
