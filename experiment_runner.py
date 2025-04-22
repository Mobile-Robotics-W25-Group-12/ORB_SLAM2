import json
from pathlib import Path
import yaml
import subprocess
import argparse

DEFAULT_PARAMS = {
    'vectorFilepath': '',
    'minVectorScore': 0.,
    'useVectorScores': False,
    'numInitialMatchPoints': 20,
    'numRansacInliers': 20,
    'numOptimizationInliers': 20,
    'projTreshold': 10,
    'numProjectedMatchPoints': 40,
    'covisibilityConsistencyTh': 3,
}

def run_experiment(experiment):
    experiment_name = experiment['name']
    config = experiment['config']
    test_cases = experiment['test_cases']
    num_trials = experiment['num_trials']

    run_command = [config['exe'], config['vocab'], config['calib'], config['dataset']]
    
    experiment_dir = Path('experiments') / experiment_name
    experiment_dir.mkdir(parents=True)
    with (experiment_dir / 'config.yaml').open('w') as f:
        yaml.safe_dump(config, f)
    with (experiment_dir / 'test_cases.json').open('w') as f:
        json.dump([test_case['name'] for test_case in test_cases], f) 

    for test_case in test_cases:
        test_case_dir = experiment_dir / test_case['name']
        test_case_dir.mkdir()

        params = {**DEFAULT_PARAMS, **test_case['params']}

        with Path('loop_closure_params.yaml').open('w') as f:
            yaml.safe_dump(params, f)
        with (test_case_dir / 'params.yaml').open('w') as f:
            yaml.safe_dump(params, f)
        
        with (test_case_dir / 'trials.json').open('w') as f:
            json.dump([str(i) for i in range(num_trials)], f)

            for i in range(num_trials):
                subprocess.run(run_command, check=True)
                log_dir = max(Path('logs').iterdir(), key=lambda f: f.name)
                log_dir.rename(test_case_dir / str(i))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run experiment.")
    parser.add_argument('experiment_file', type=str, help="Path to the experiment JSON file.")
    args = parser.parse_args()

    with open(args.experiment_file, 'r') as f:
        experiment = json.load(f)

    run_experiment(experiment)
