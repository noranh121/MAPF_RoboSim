import subprocess
import json
import a_star_algorithm
# result = subprocess.run(
#     ["python3", "src/mapf_simulator/backend/backend/a_star_algorithm.py","benchmark.txt","test.txt"],
#     capture_output=True,
#     text=True
# )

# if result.stderr:
#     print(f"Error in subprocess: {result.stderr}")
# else:
#     try:
#         # Attempt to load the JSON from stdout
#         waypoints = json.loads(result.stdout.strip())
#         print("Output from a_star_algorithm:", waypoints)
#     except json.JSONDecodeError:
#         print("Failed to decode JSON.")
#         print("Raw output:", result.stdout)

rsult=a_star_algorithm.main("benchmark.txt","test.txt")
print(rsult)