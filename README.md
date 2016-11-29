Matlab code for ACC paper

## To regenerate the figures for publications

### Configuration Error Function visualization:
run log_barrier.m to make the surface plots for the \Psi

Go to commit 21e53de0fb288911a5dd5306fddb1f8a9d6d5a8d to make sure the constants
are setup properly

### Attitude Stabilization without adaptive update law:
run coupled_control_driver.m and then run plot_outputs.m to generate the simulation
plots used in both ACC/IJCAS submission

This is using commit 3c428392a28a19de57c34fbe98816d4e6024d7ff

### Attitude Stabilization with adaptive update law:
run coupled_control_driver.m and then run plot_outputs.m or draw_cad to generate
the plots. There's a flag in load_constants to create animations or a video

Go to commit c44b078d2ad42443ac2875aa09a726a987e7af12 to ensure the constants 
are setup properly

### Constrained Attitude stabilization experiment:
Run Data_analysis.m to read 20150924_avoid3.txt to generate the experimental results

The repository holding the experimental data is located at
https://github.com/skulumani/2016_ACC_Experiment

The commit is 6f5604a9db17ee67b019dac6daa1807e5ed3c6d5

This calls load_experiment_constants and plot_experiment_constants 