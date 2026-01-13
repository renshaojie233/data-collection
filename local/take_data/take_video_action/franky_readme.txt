实现类/home/ubuntu/take_data/take_video_action/franky_remote_client.py
##########################################################################################
最简调用/home/ubuntu/take_data/take_video_action/franky_test.py

##########################################################################################
画图测试/home/ubuntu/take_data/take_video_action/franky_remote_plot_joint7_curve.py
保存结果在/home/ubuntu/take_data/take_video_action/joint7_curve_remote.png
第七个关节角，有些许滞后，但是幅度是对的

##########################################################################################
读json位置，发送当前状态+json前后两个位置之差/home/ubuntu/take_data/take_video_action/franky_remote_delta_json.py
能执行、有误差，可能要调dynamics_factor。 作者恢复不连续问题：https://github.com/TimSchneider42/franky/issues/74

##########################################################################################
环境 conda activate take_data
cd /home/ubuntu/take_data/take_video_action
python franky_test.py
