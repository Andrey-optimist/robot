# How to run:

copy directoty priv2 to your workspace

rosrun priv2 tf_broadcaster 

roslaunch priv2 priv2_gazebo.launch localization_type:='AMCL' map:=map

rosrun priv2 navigation_goals
