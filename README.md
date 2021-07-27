# How to run:

copy directoty priv2 to your workspace

roslaunch priv2 priv2_gazebo.launch localization_type:='AMCL' map:=map

rosrun priv2 tf_broadcaster 

rosrun priv2 navigation_goals_test
