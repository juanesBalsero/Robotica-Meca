<?xml version="1.0"?>
<launch>

  <!-- Ruta al URDF -->
  <arg name="model" default="$(find Completo)/urdf/Completo.urdf" />
  <arg name="rvizconfig" default="$(find Completo)/rviz/config.rviz" />

  <!-- Carga el URDF al parametro robot_description -->
  <param name="robot_description" command="cat $(arg model)" />

  <!-- Publica sliders para mover cada joint (J1, J2, J3) -->
  <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui"
        type="joint_state_publisher_gui" />

  <!-- Publica las transformadas TF de cada link segun el estado de los joints -->
  <node name="robot_state_publisher" pkg="robot_state_publisher"
        type="robot_state_publisher" />

  <!-- Abre RViz ya configurado -->
  <node name="rviz" pkg="rviz" type="rviz"
        args="-d $(arg rvizconfig)" required="true" />

</launch>
