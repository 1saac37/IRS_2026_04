read me

GIT folder: IRS_2026

main project: final_pick_place

hand_solo_arm, hand_solo_virtual_nav, pa_warehouse_status are drafts 

components of final_pick_place:

- final_pick_place
    - pick_place.py

- launch
    - mapping_launch
    - nav_launch

- map
- config
    - yaml files
- resource
- rviz
- setup.cfg
- setup.py

commands to launch

cd irs_ws

colcon build

source install/local_setup.bash

ros2 launch final_pick_place nav_launch.py

in another terminal

ros2 run final_pick_place pick_place