import json
from pathlib import Path
import yaml
import subprocess

DEFAULT_PARAMS = {
    'vectorFilepath': '',
    'minVectorScore': 0.,
    'useVectorScores': False,
    'numInitialMatchPoints': 20,
    'numRansacInliers': 20,
    'numOptimizationInliers': 20,
    'projTreshold': 10,
    'numProjectedMatchPoints': 40,
}

config = {
    'exe': './Examples/Monocular/mono_kitti',
    'vocab': 'Vocabulary/ORBvoc.txt',
    'calib': 'Examples/Monocular/KITTI04-12.yaml',
    'dataset': 'datasets/kitti_grey/sequences/06_brightness',
}

experiment_name = 'brightness'

test_cases = [
    {
        'name': 'bow',
        'params': {
            'useVectorScores': False,
            'minVectorScore': 0.0,

        }
    },
    {
        'name': 'boq',
        'params': {
            'useVectorScores': True,
            'minVectorScore': 0.6,
            'vectorFilepath': '06_brightness.npy'
        }
    },
]

def get_run_command():
    return [config['exe'], config['vocab'], config['calib'], config['dataset']]

num_trials = 5

experiment_dir = Path('experiments') / experiment_name
experiment_dir.mkdir(parents=True, exist_ok=True)
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
        subprocess.run(get_run_command(), check=True)
        log_dir = max(Path('logs').iterdir(), key=lambda f: f.name)
        log_dir.rename(test_case_dir / str(i))
    