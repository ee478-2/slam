# slam_aliases.sh — short commands for the perception/SLAM stack.
#
#   source ~/catkin_ws/src/slam/scripts/slam_aliases.sh
#   slam up        # env + arm home + camera + wheel odom + rtabmap + apriltag
#   slam collect-cam # camera-only data collection; no rtab/apriltag/localization
#   slam collect   # save storefront YOLO images while teleop drives
#   slam help      # list everything
#
# Add the source line to ~/.bashrc to get it in every shell.
#
# Overridable: SLAM_WS (workspace), SLAM_PI (Pi roscore IP), SLAM_DISPLAY (GUI :N).

SLAM_WS="${SLAM_WS:-$HOME/catkin_ws}"
SLAM_PI="${SLAM_PI:-192.168.0.200}"
SLAM_DISPLAY="${SLAM_DISPLAY:-:1}"

slam() {
  local cmd="${1:-help}"; shift 2>/dev/null
  case "$cmd" in
    env)
      source "$SLAM_WS/devel/setup.bash"
      export ROS_MASTER_URI="http://$SLAM_PI:11311"
      export ROS_IP="$(ip -4 -o addr show eth0 2>/dev/null | awk '{print $4}' | cut -d/ -f1)"
      echo "MASTER=$ROS_MASTER_URI  IP=$ROS_IP" ;;

    cam)   # RealSense, USB2 low-bandwidth recipe (SIGINT-only to stop!)
      if pgrep -f 'rs_camera.launch' >/dev/null 2>&1; then
        echo "camera already running — run 'slam down' first (else it races device-busy)"; return 1; fi
      setsid roslaunch realsense2_camera rs_camera.launch \
        enable_depth:=true align_depth:=true enable_color:=true \
        enable_sync:=true enable_pointcloud:=true \
        enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false \
        color_width:=640 color_height:=480 color_fps:=15 \
        depth_width:=640 depth_height:=480 depth_fps:=15 >/tmp/rs_camera.log 2>&1 &
      echo "camera (USB2 640x480@15 + pointcloud) -> /tmp/rs_camera.log" ;;

    rtab)  local guess="${SLAM_ODOM_GUESS_FRAME:-}"
           setsid roslaunch slam rtabmap_realsense.launch rviz:=false rtabmap_viz:=false \
             odom_guess_frame_id:="$guess" >/tmp/rtabmap.log 2>&1 &
           if [ -n "$guess" ]; then
             echo "rtabmap (odom guess=$guess) -> /tmp/rtabmap.log"
           else
             echo "rtabmap -> /tmp/rtabmap.log"
           fi ;;

    rtab-map|rtab_map|map-db|map_db)
           local db="${SLAM_RTAB_DB:-$HOME/.ros/storefront_rtabmap.db}"
           local reset="${SLAM_RTAB_RESET:-true}"
           local args="--Reg/Force3DoF true"
           if [ "$reset" = "true" ] || [ "$reset" = "1" ]; then
             args="--delete_db_on_start $args"
           fi
           setsid roslaunch slam rtabmap_realsense.launch rviz:=false rtabmap_viz:=false \
             localization:=false database_path:="$db" rtabmap_args:="$args" \
             >/tmp/rtabmap_map.log 2>&1 &
           echo "rtabmap mapping DB=$db reset=$reset -> /tmp/rtabmap_map.log" ;;

    rtab-loc|rtab_loc|localize-db|localize_db)
           local db="${SLAM_RTAB_DB:-$HOME/.ros/storefront_rtabmap.db}"
           setsid roslaunch slam rtabmap_realsense.launch rviz:=false rtabmap_viz:=false \
             localization:=true database_path:="$db" \
             rtabmap_args:="--Reg/Force3DoF true" >/tmp/rtabmap_loc.log 2>&1 &
           echo "rtabmap localization DB=$db -> /tmp/rtabmap_loc.log" ;;

    wheel|wheel-odom|wheel_odom)
           setsid roslaunch slam wheel_odom.launch >/tmp/wheel_odom.log 2>&1 & \
             echo "wheel odom (/wheel/odom, no TF) -> /tmp/wheel_odom.log" ;;

    wheel-tf|wheel_tf)
           setsid roslaunch slam wheel_odom.launch publish_tf:=true \
             >/tmp/wheel_odom.log 2>&1 & \
             echo "wheel odom (/wheel/odom + TF; use carefully with RTAB TF) -> /tmp/wheel_odom.log" ;;

    tags)  setsid roslaunch slam apriltag_realsense.launch >/tmp/apriltag.log 2>&1 & \
             echo "apriltag -> /tmp/apriltag.log" ;;

    yolo-tags|yolo_tags|pose-tags|pose_tags)
           local model="${SLAM_YOLO_POSE_MODEL:-$SLAM_WS/src/slam/pose_best.engine}"
           local hz="${SLAM_YOLO_POSE_HZ:-5.0}"
           local output="${SLAM_YOLO_POSE_OUTPUT:-/tag_detections}"
           local debug_image="${SLAM_YOLO_POSE_DEBUG_IMAGE:-/yolo_pose_tag_detector/debug_image}"
           local publish_debug_image="${SLAM_YOLO_POSE_PUBLISH_DEBUG_IMAGE:-true}"
           local min_box_conf="${SLAM_YOLO_POSE_MIN_BOX_CONF:-0.50}"
           local min_keypoint_conf="${SLAM_YOLO_POSE_MIN_KEYPOINT_CONF:-0.45}"
           local stable_frames="${SLAM_YOLO_POSE_MIN_STABLE_FRAMES:-5}"
           local ema="${SLAM_YOLO_POSE_EMA_ALPHA:-0.35}"
           local class_map="${SLAM_YOLO_POSE_CLASS_ID_TO_TAG_ID:-}"
           setsid roslaunch slam yolo_pose_tag_detector.launch \
             model_path:="$model" inference_hz:="$hz" output_topic:="$output" \
             debug_image_topic:="$debug_image" publish_debug_image:="$publish_debug_image" \
             min_box_conf:="$min_box_conf" min_keypoint_conf:="$min_keypoint_conf" \
             min_stable_frames:="$stable_frames" ema_alpha:="$ema" \
             class_id_to_tag_id:="$class_map" >/tmp/yolo_pose_tags.log 2>&1 & \
             echo "yolo pose square tags -> /tmp/yolo_pose_tags.log"
           echo "  model: $model"
           echo "  output: $output (RTAB consumes /tag_detections by default)"
           echo "  debug image: $debug_image" ;;

    yolo-tags-remote|yolo_tags_remote|remote-yolo-tags|remote_yolo_tags)
           if [ ! -f "$SLAM_WS/devel/setup.bash" ]; then
             echo "missing workspace setup: $SLAM_WS/devel/setup.bash"; return 1
           fi
           source "$SLAM_WS/devel/setup.bash"
           export ROS_MASTER_URI="http://$SLAM_PI:11311"
           local route_ip
           route_ip="$(ip route get "$SLAM_PI" 2>/dev/null | awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}')"
           export ROS_IP="${SLAM_REMOTE_ROS_IP:-${route_ip:-${ROS_IP:-$(hostname -I | awk '{print $1}')}}}"
           unset ROS_HOSTNAME
           local default_model="$SLAM_WS/src/slam/pose_best.pt"
           if [ -z "${SLAM_REMOTE_YOLO_POSE_MODEL:-}" ] && [ ! -f "$default_model" ] && [ -f "$SLAM_WS/src/slam/pose_best.onnx" ]; then
             default_model="$SLAM_WS/src/slam/pose_best.onnx"
           fi
           local model="${SLAM_REMOTE_YOLO_POSE_MODEL:-$default_model}"
           local image_topic="${SLAM_REMOTE_YOLO_POSE_IMAGE_TOPIC:-${SLAM_YOLO_POSE_IMAGE_TOPIC:-/camera/color/image_raw}}"
           local camera_info_topic="${SLAM_REMOTE_YOLO_POSE_CAMERA_INFO_TOPIC:-${SLAM_YOLO_POSE_CAMERA_INFO_TOPIC:-/camera/color/camera_info}}"
           local default_imgsz=640
           case "$model" in
             *.onnx) default_imgsz=512 ;;
           esac
           local imgsz="${SLAM_REMOTE_YOLO_POSE_IMGSZ:-$default_imgsz}"
           local hz="${SLAM_REMOTE_YOLO_POSE_HZ:-${SLAM_YOLO_POSE_HZ:-5.0}}"
           local output="${SLAM_REMOTE_YOLO_POSE_OUTPUT:-${SLAM_YOLO_POSE_OUTPUT:-/tag_detections}}"
           local debug_image="${SLAM_REMOTE_YOLO_POSE_DEBUG_IMAGE:-${SLAM_YOLO_POSE_DEBUG_IMAGE:-/yolo_pose_tag_detector/debug_image}}"
           local publish_debug_image="${SLAM_REMOTE_YOLO_POSE_PUBLISH_DEBUG_IMAGE:-${SLAM_YOLO_POSE_PUBLISH_DEBUG_IMAGE:-true}}"
           local min_box_conf="${SLAM_REMOTE_YOLO_POSE_MIN_BOX_CONF:-${SLAM_YOLO_POSE_MIN_BOX_CONF:-0.50}}"
           local min_keypoint_conf="${SLAM_REMOTE_YOLO_POSE_MIN_KEYPOINT_CONF:-${SLAM_YOLO_POSE_MIN_KEYPOINT_CONF:-0.45}}"
           local stable_frames="${SLAM_REMOTE_YOLO_POSE_MIN_STABLE_FRAMES:-${SLAM_YOLO_POSE_MIN_STABLE_FRAMES:-5}}"
           local ema="${SLAM_REMOTE_YOLO_POSE_EMA_ALPHA:-${SLAM_YOLO_POSE_EMA_ALPHA:-0.35}}"
           local class_map="${SLAM_REMOTE_YOLO_POSE_CLASS_ID_TO_TAG_ID:-${SLAM_YOLO_POSE_CLASS_ID_TO_TAG_ID:-}}"
           if [ -z "$ROS_IP" ]; then
             echo "could not infer ROS_IP; set SLAM_REMOTE_ROS_IP=<laptop_ip>"; return 1
           fi
           if [ ! -f "$model" ]; then
             echo "remote YOLO model missing: $model"
             echo "copy pose_best.pt or pose_best.onnx to the laptop, or set SLAM_REMOTE_YOLO_POSE_MODEL"
             return 1
           fi
           setsid roslaunch slam yolo_pose_tag_detector.launch \
             model_path:="$model" image_topic:="$image_topic" camera_info_topic:="$camera_info_topic" \
             imgsz:="$imgsz" inference_hz:="$hz" output_topic:="$output" \
             debug_image_topic:="$debug_image" publish_debug_image:="$publish_debug_image" \
             min_box_conf:="$min_box_conf" min_keypoint_conf:="$min_keypoint_conf" \
             min_stable_frames:="$stable_frames" ema_alpha:="$ema" \
             class_id_to_tag_id:="$class_map" >/tmp/yolo_pose_tags_remote.log 2>&1 & \
             echo "remote yolo pose square tags -> /tmp/yolo_pose_tags_remote.log"
           echo "  MASTER=$ROS_MASTER_URI  IP=$ROS_IP"
           echo "  route to Pi: $(ip route get "$SLAM_PI" 2>/dev/null || echo unavailable)"
           echo "  model: $model"
           echo "  image: $image_topic"
           echo "  camera info: $camera_info_topic"
           echo "  imgsz: $imgsz"
           echo "  output: $output (RTAB consumes /tag_detections)"
           echo "  debug image: $debug_image" ;;

    global-loc|global_loc)
           local stable_frames="${SLAM_APRILTAG_MIN_STABLE_FRAMES:-3}"
           local smoothing_window="${SLAM_APRILTAG_SMOOTHING_WINDOW:-5}"
           local normal_heading="${SLAM_APRILTAG_USE_NORMAL_HEADING:-true}"
           local max_inplane_correction="${SLAM_APRILTAG_MAX_INPLANE_CORRECTION_DEG:-60.0}"
           local single_landmark_yaw="${SLAM_APRILTAG_SINGLE_LANDMARK_YAW_SOURCE:-hold}"
           setsid roslaunch slam apriltag_global_localization.launch \
             min_stable_frames:="$stable_frames" \
             smoothing_window_size:="$smoothing_window" \
             use_tag_normal_heading:="$normal_heading" \
             max_tag_inplane_yaw_correction_deg:="$max_inplane_correction" \
             single_landmark_yaw_source:="$single_landmark_yaw" \
             >/tmp/global_loc.log 2>&1 & \
             echo "apriltag global localization (global_map->map, stable frames=$stable_frames, smoothing window=$smoothing_window, normal heading=$normal_heading, max in-plane correction=${max_inplane_correction}deg, single landmark yaw=$single_landmark_yaw) -> /tmp/global_loc.log" ;;

    loc)   # localization_manager: fuses tag/rtabmap pose -> /robot_pose + /odom
           setsid roslaunch slam localization_manager.launch >/tmp/locman.log 2>&1 & \
             echo "localization_manager (/odom + /robot_pose) -> /tmp/locman.log" ;;

    arm-home|arm_home)
           local pose="${SLAM_ARM_HOME_POSE:-0 0.8 -3. -0.5 0}"
           local gripper="${SLAM_GRIPPER_HOME_POSITION:--1.20}"
           setsid roslaunch slam arm_home.launch home_pose:="$pose" \
             gripper_position:="$gripper" >/tmp/arm_home.log 2>&1 & \
             echo "arm home pose -> /tmp/arm_home.log" ;;

    yolo)  setsid rosrun manipulation_control object_detection.py \
             _base_frame:=camera_link _visualize:=true _inference_hz:=8.0 \
             >/tmp/yolo_det.log 2>&1 & echo "yolo -> /tmp/yolo_det.log" ;;

    collect|collect-data|storefront-data)
           local out="${SLAM_STORE_YOLO_RAW:-$SLAM_WS/src/slam/data/storefront_yolo/raw}"
           local hz="${SLAM_STORE_YOLO_HZ:-2.0}"
           local session="${SLAM_STORE_YOLO_SESSION:-}"
           setsid roslaunch slam storefront_data_collection.launch \
             output_root:="$out" capture_hz:="$hz" session:="$session" \
             >/tmp/storefront_collect.log 2>&1 & \
             echo "storefront image collector -> /tmp/storefront_collect.log"
           echo "  output: $out"
           echo "  drive with: slam teleop" ;;

    collect-cam|collect_cam|data-cam|data_cam)
           slam env
           if ! pgrep -f 'rs_camera.launch' >/dev/null 2>&1; then
             slam cam
             echo "...waiting for camera color stream"
             local i; for i in $(seq 1 20); do
               timeout 2 rostopic echo -n1 /camera/color/image_raw/header >/dev/null 2>&1 && { echo "camera streaming"; break; }
               sleep 1
             done
             timeout 2 rostopic echo -n1 /camera/color/image_raw/header >/dev/null 2>&1 || { echo "camera did not stream; tail /tmp/rs_camera.log"; tail -n 80 /tmp/rs_camera.log; return 1; }
           else
             echo "camera already running"
           fi
           slam collect
           echo "camera-only collection running; no RTAB/AprilTag/localization started" ;;

    teleop) rosrun slam teleop_keyboard.py ;;   # foreground, needs a real keyboard

    rviz)  local cfg="${1:-apriltag_rtabmap}"   # or: slam rviz yolo
      DISPLAY="$SLAM_DISPLAY" setsid rviz -d "$SLAM_WS/src/slam/rviz/$cfg.rviz" \
        >/tmp/rviz.log 2>&1 & echo "rviz [$cfg] on $SLAM_DISPLAY" ;;

    odom-viz|odom_viz|wheel-viz|wheel_viz)
      DISPLAY="$SLAM_DISPLAY" setsid roslaunch slam rtab_wheel_viz.launch \
        >/tmp/rtab_wheel_viz.log 2>&1 & \
        echo "rtab/wheel odom RViz -> /tmp/rtab_wheel_viz.log" ;;

    mission) slam env
      DISPLAY="$SLAM_DISPLAY" setsid roslaunch slam mission_viz.launch \
        >/tmp/mission_viz.log 2>&1 & echo "mission viz -> /tmp/mission_viz.log" ;;

    mission-pub|mission_pub) slam env
      setsid roslaunch slam mission_viz.launch run_rviz:=false \
        >/tmp/mission_viz.log 2>&1 & echo "mission markers only -> /tmp/mission_viz.log" ;;

    up)    slam env; slam arm-home; slam cam
           echo "...waiting for camera to actually stream"
           local i; for i in $(seq 1 20); do
             timeout 2 rostopic echo -n1 /camera/color/image_raw/header >/dev/null 2>&1 && { echo "camera streaming"; break; }
             sleep 1; done
           echo "...waiting for pointcloud"
           for i in $(seq 1 20); do
             timeout 2 rostopic echo -n1 /camera/depth/color/points/header >/dev/null 2>&1 && { echo "pointcloud streaming"; break; }
             sleep 1; done
           timeout 2 rostopic echo -n1 /camera/depth/color/points/header >/dev/null 2>&1 || { echo "pointcloud did not stream; tail /tmp/rs_camera.log"; tail -n 80 /tmp/rs_camera.log; return 1; }
           slam wheel; slam rtab; slam tags; slam global-loc; slam loc
           echo "stack up (arm home+camera+wheel odom+rtabmap+apriltag+global localization+localization). /wheel/odom + /odom + /robot_pose"
           echo "  /odom is global_map when a known signboard tag anchors RTAB; otherwise it falls back to RTAB odom"
           echo "  check: rostopic echo -n1 /global_localization/selected_tag && rostopic echo -n1 /odom/header"
           echo "NOTE: RViz is SEPARATE -> 'slam rviz' (on :1), or run rviz on your PC." ;;

    mon)   python3 - <<'PY'
import rospy,time,math
from rtabmap_msgs.msg import MapData,Info
from nav_msgs.msg import Odometry
rospy.init_node('slammon',anonymous=True,disable_signals=True)
s={'n':0,'lc':0,'lost':0,'good':0,'p':None,'d':0.0}
def md(m): s['n']=len(m.graph.poses)
def inf(m):
    if getattr(m,'loopClosureId',0)>0: s['lc']+=1
def od(m):
    s['lost' if m.pose.covariance[0]>=9998 else 'good']+=1
    p=m.pose.pose.position
    if s['p']: s['d']+=math.dist((p.x,p.y,p.z),s['p'])
    s['p']=(p.x,p.y,p.z)
rospy.Subscriber('/rtabmap/mapData',MapData,md)
rospy.Subscriber('/rtabmap/info',Info,inf)
rospy.Subscriber('/rtabmap/odom',Odometry,od)
print("monitoring 15s..."); time.sleep(15)
print(f"nodes={s['n']} loopclose={s['lc']} VO good/lost={s['good']}/{s['lost']} travel~{s['d']:.2f}m")
PY
      ;;

    check) echo "== running ==";
      ps -eo pid,comm | awk '$2~/nodelet|rgbd_odometry|rtabmap|apriltag|rviz|object_dete/{print "  "$0}'
      python3 - <<'PY'
import rospy,time
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray
from rtabmap_msgs.msg import MapData
rospy.init_node('slamchk',anonymous=True,disable_signals=True)
s={'ids':set(),'lm':0,'n':0}
def td(m):
    for d in m.detections: s['ids'].add(d.id[0] if len(d.id)==1 else tuple(d.id))
rospy.Subscriber('/tag_detections',AprilTagDetectionArray,td)
rospy.Subscriber('/rtabmap/landmarks',PoseArray,lambda m:s.__setitem__('lm',len(m.poses)))
rospy.Subscriber('/rtabmap/mapData',MapData,lambda m:s.__setitem__('n',len(m.graph.poses)))
time.sleep(5)
print(f"== rtabmap: map_nodes={s['n']} landmarks={s['lm']} | tags seen(5s)={sorted(s['ids']) or 'none'}")
PY
      ;;

    down)  # SIGINT-only for the camera; consumers first
      pkill -INT -f 'rtabmap_viz/rtabmap_viz' 2>/dev/null
      pkill -INT -f 'roslaunch slam rtab_wheel_viz' 2>/dev/null
      pkill -INT -f 'roslaunch slam localization_manager' 2>/dev/null
      pkill -INT -f 'roslaunch slam apriltag_global_localization' 2>/dev/null
      pkill -INT -f 'roslaunch slam wheel_odom' 2>/dev/null
      pkill -INT -f 'roslaunch slam storefront_data_collection' 2>/dev/null
      pkill -INT -f 'storefront_dataset_collector.py' 2>/dev/null
      pkill -INT -f 'roslaunch slam yolo_pose_tag_detector' 2>/dev/null
      pkill -INT -f 'yolo_pose_tag_detector.py' 2>/dev/null
      pkill -INT -f 'roslaunch slam apriltag_realsense' 2>/dev/null
      pkill -INT -f 'roslaunch slam rtabmap_realsense' 2>/dev/null
      pkill -f 'object_detection.py' 2>/dev/null
      ps -eo pid,comm | awk '$2=="rviz"{print $1}' | xargs -r kill 2>/dev/null
      sleep 5
      pkill -INT -f 'roslaunch realsense2_camera' 2>/dev/null
      # WAIT for the realsense nodelet to release the USB device before returning,
      # else a quick 'slam up' races device-busy and the camera nodelet dies (0 Hz topics).
      local i; for i in $(seq 1 12); do pgrep -f 'rs_camera.launch' >/dev/null 2>&1 || break; sleep 1; done
      echo "stack down, camera released. (verify: ps -eo pid,comm | grep -E 'nodelet|rtabmap')" ;;

    help|*) cat <<EOF
slam <cmd>:
  env           source workspace + set ROS master/IP
  up            env + arm-home + cam + wheel + rtab + tags + loc  (full bring-up)
  cam           RealSense camera (USB2 recipe)
  arm-home      open gripper + move arm to default home pose
  wheel         /wheel/odom from chassis commands (no TF by default)
  wheel-tf      /wheel/odom plus wheel_odom->base_link TF (isolated tests)
  rtab          rtabmap RGB-D SLAM
  tags          apriltag detection (+ rtabmap landmarks)
  yolo-tags     YOLO pose square tags as RTAB-compatible landmarks
  yolo-tags-remote
                laptop-side YOLO pose tags; subscribes camera, publishes /tag_detections
  global-loc    AprilTag anchor: publish global_map->map + global robot pose
  loc           localization_manager -> /robot_pose + /odom (global if anchored)
  yolo          YOLO object detection
  rtab-map      build/update RTAB feature DB (SLAM_RTAB_DB, SLAM_RTAB_RESET)
  rtab-loc      localize against RTAB feature DB without adding map nodes
  collect-cam   env + camera + collect only (no RTAB/AprilTag/localization)
  collect       save storefront YOLO images while teleop drives
  teleop        keyboard teleop (foreground)
  rviz [name]   rviz on $SLAM_DISPLAY (default apriltag_rtabmap; or: slam rviz yolo)
  odom-viz      RViz compare: RTAB green, wheel odom red (no AprilTag displays)
  mission       mission marker publisher + RViz total mission view on $SLAM_DISPLAY
  mission-pub   mission marker publisher only (use this for laptop RViz)
  mon           monitor map / loop-closure / VO for 15s
  check         quick status snapshot
  down          SIGINT teardown (camera stopped LAST)
EOF
      ;;
  esac
}
