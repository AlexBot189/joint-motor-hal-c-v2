rm img/*
sync
cp motor_hal/build/libmotor_hal.so img
cp motor_hal/build/hal_stress_test img
cp motor_hal/tools/build/motor_tool img
cp exo_node/build/stark_periph_manager_node img
cp exo_node/test/build/algo_sim img
cp exo_node/test/build/algo_sim_single img
cp exo_node/test/build/perf_test img
cp exo_node/config/exo_config_single.json img
sync
scp -r img ecoyzq@192.168.0.114:/home/workspace/01-proj/stark/test_demo
