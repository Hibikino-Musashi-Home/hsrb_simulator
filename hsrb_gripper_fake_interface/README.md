`hsrb_gripper_fake_interface`
===============================================================================

概要
-------------------------------------------------------------------------------

本パッケージは、`ros2-control`上で動作するシミュレータのhandを制御するhardware_interfaceとなります。

-------------------------------------------------------------------------------

メインノード。
本ノードは以下の処理を実施します。　　

- **on_init**
各パラメータの取得と、変数の初期化を実施します。

- **export_state_interfaces**
ステート用に以下のhardware_interface項目を作成します。
本項目は、**hsrb_gripper_controller**側で参照されます。
    - **hand_motor_joint_bridge/position**：hand_motor_jointのposition値
    - **hand_motor_joint_bridge/velocity**：hand_motor_jointのvelocity値
    - **hand_motor_joint_bridge/effort**：hand_motor_jointのeffort値
    - **hand_motor_joint_bridge/current_drive_mode**：現在のdriveモード値
    - **hand_motor_joint_bridge/current_grasping_flag**：現在のgraspingフラグ値
    - **hand_motor_joint_bridge/current**：現在の電流値

- **export_command_interfaces**
コマンド用に以下のhardware_interface項目を作成します。
本項目は、**hsrb_gripper_controller**側から指示されます。
    - **hand_motor_joint_bridge/position**：hand_motor_jointへのposition指令値
    - **hand_motor_joint_bridge/velocity**：hand_motor_jointへのvelocity指令値 ※未使用
    - **hand_motor_joint_bridge/effort**：hand_motor_jointのeffort指令値 ※未使用
    - **hand_motor_joint_bridge/command_drive_mode**：driveモード指令値
    - **hand_motor_joint_bridge/command_grasping_flag**：graspingフラグ指令値

- **write**
graspingフラグの指令値と現在値を比較し、異なった場合且つ、graspingフラグの指令値が1の場合、
gripperの全開、全閉制御を実施します。全開、全閉制御の条件は以下の通りとなります。
また、position指令値とposition指令値の前回値を比較し異なっていた場合は、**hand_motor_joint**へ
positionの指示を行います。
<全開・全閉の条件について>
    - effortの絶対値が閾値より大きい場合で、effort値が+ : 全開
    - effortの絶対値が閾値より大きい場合で、effort値が- : 全閉


#### パラメータ

- `effort_min : double`

    effortの閾値 default : 0.011

### 使用方法(プラグインの組み込み方法について)
既にhardware interfaceのプラグインとしては組み込み済みですが、使用方法は以下の通りとなります。

  - hsrb_common/hsrb_description/urdf/ros2_control.rviz.xacroをオープンして下さい。
  - 以下のタグを追加して下さい。

``` xml
    <ros2_control name="HsrbGripperFakeInterface" type="system">
      <hardware>
        <plugin>hsrb_gripper_fake_interface/HsrbGripperFakeInterface</plugin>
        <!-- effortの閾値 -->
        <param name="effort_min">0.011</param>
      </hardware>

      <joint name="hand_motor_joint">
      </joint>
    </ros2_control>

```

### 使用方法(起動方法について)

  - 以下のコマンドでrvizのlaunchを起動させて下さい。
```shell-session
      $ ros2 launch hsrb_rviz_simulator hsrb_rviz_simulator.launch.py
```

  - 起動後に初期姿勢の遷移が動きますので、この時にグリッパが開けば本パッケージは正常に
　　組み込まれています。

### 確認方法

  - 以下のコマンドでグリッパを操作して下さい。positionを0.3に指定しています。
```shell-session
      $ ros2 topic pub --once  /safe_pose_changer/joint_reference sensor_msgs/msg/JointState "{name: ['hand_motor_joint'], position: [0.3], velocity: [0.0], effort: [0.0]}"
```
  - rivz上のグリッパが動作する事をご確認下さい。
  - 以下のコマンドでグリッパのgraspを動作させて下さい。
```shell-session
      $ ros2 action send_goal /gripper_controller/grasp tmc_control_msgs/action/GripperApplyEffort "{ effort: 0.0,do_control_stop: false}"
```
  - rivz上のグリッパが閉じる事を確認下さい。
