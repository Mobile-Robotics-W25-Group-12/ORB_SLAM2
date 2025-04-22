import experiment_runner

def run_experiment(name, sequence, vectors, num_trials, dataset="kitti_grey", exe="mono_kitti", calib="KITTI04-12.yaml", common_params = {}, bow_params = {}, boq_params = {}, exclude_boq = False, exclude_bow = False):
    experiment = {
        "name": name,
        "num_trials": num_trials,
        "config": {
            "exe": f"./Examples/Monocular/{exe}",
            "vocab": "Vocabulary/ORBvoc.txt",
            "calib": f"Examples/Monocular/{calib}",
            "dataset": f"datasets/{dataset}/sequences/{sequence}/"
        },
        "test_cases": []
    }
    if not exclude_boq:
        experiment["test_cases"].append({
            "name": "boq",
            "params": {
                "useVectorScores": True,
                "minVectorScore": 0.6,
                "vectorFilepath": vectors,
                **common_params,
                **boq_params
            }
        })
    if not exclude_bow:
        experiment["test_cases"].append({
            "name": "bow",
            "params": {
                "useVectorScores": False,
                "minVectorScore": 0.0,
                **common_params,
                **bow_params
            }
        })
    experiment_runner.run_experiment(experiment)

if __name__ == '__main__':
    run_experiment('06_brightness', '06_brightness', '06_brightness.npy', 10)
    run_experiment('05_brightness', '05_brightness', '05_brightness.npy', 10)
    run_experiment('05_brightness_0.4', '05_brightness', '05_brightness.npy', 10, boq_params={'minVectorScore': 0.4}, exclude_bow=True)
    run_experiment('05_brightness_0.4_no_consistency', '05_brightness', '05_brightness.npy', 10, common_params={'covisibilityConsistencyTh': 1}, 
                   boq_params={'minVectorScore': 0.4})

    run_experiment('06_brightness_0.4', '06_brightness', '06_brightness.npy', 10, boq_params={'minVectorScore': 0.4}, exclude_bow=True)
    run_experiment('06_brightness_0.4_no_consistency', '06_brightness', '06_brightness.npy', 10, common_params={'covisibilityConsistencyTh': 1}, 
                   boq_params={'minVectorScore': 0.4})

    # Use this for iphone experiments
    # run_experiment('iphone_06', '06', 'iphone_06_final.npy', 10, 
    #     dataset='iphone', exe='mono_iphone', calib="iphone.yaml",
    #     common_params={
    #         'numInitialMatchPoints': 5,
    #         'numOptimizationInliers': 5,
    #         'numRansacInliers': 5
    #     },
    #     boq_params={
    #         'minVectorScore': 0.4,
    #     }
    # )

