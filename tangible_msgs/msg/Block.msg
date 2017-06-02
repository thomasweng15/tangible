float32 EDGE_SIZE=0.05
float32 ARROW_SELECTION_LEN=0.13
float32 CORNER_SELECTION_LEN=0.08
# ID of selection tags {0, 1, 2, 3}
uint8 SELECTION_ID_MIN=0
uint8 SELECTION_ID_MAX=3
uint8 SELECT_POSITION_ID=0
uint8 SELECT_REGION_ID=1
uint8 SELECT_OBJECT_ID=2
uint8 SELECT_OBJECTS_ID=3
# ID of secondary selection tag=4
uint8 SELECTION_2ND_ID=4
# ID of action tags {5, 6, 7, 8}
uint8 ACTION_ID_MIN=5
uint8 ACTION_ID_MAX=8
uint8 SIDE_PICK_ID=5
uint8 TOP_PICK_ID=6
uint8 POSITION_ID=7
uint8 DROP_ID=8
# ID of number tags {9, 10, ..., 15}
uint8 NUMBER_ID_MIN=9
uint8 NUMBER_ID_MAX=15
geometry_msgs/PoseStamped pose
geometry_msgs/Vector3 x_axis
geometry_msgs/Vector3 y_axis
geometry_msgs/Vector3 z_axis
uint8 id