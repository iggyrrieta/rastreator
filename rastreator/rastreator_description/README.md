# Create XACRO

The easiest and fastest way to create a robot model is using **XACRO**. This is a macro-based lenguage that makes the xml format much easier.

`xacro` file is pretty basic, the following can be used as a basic example to fill in: 

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="rastreator">
    <!--LINK 1-->
    <link name="name1">
      <inertial>
        <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
        <mass value="0.0"/>
        <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
      </inertial>
      <visual>
        ...
      </visual>
      <collision>
        ...
      </collision>
    </link>
    <!--LINK 2-->
    <link name="name2">
      <inertial>
        <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
        <mass value="0.0"/>
        <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
      </inertial>
      <visual>
        ...
      </visual>
      <collision>
        ...
      </collision>
    </link>
    <!--JOINT BETWEEN LINK 1 AND LINK 2-->
    <joint name="joint_1_2" type="fixed"> <!--THIS CAN BE FIXED, CONTINOUS OR REVOLUTE-->
        <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
        <parent link="name1"/>
        <child link="name2"/>
    </joint>
</robot>
```

To use the macro properties:

```xml
<!--base_link global variables-->
<xacro:property name="base_mass" value="2.0"/>
<xacro:property name="base_width" value="0.175"/>
<xacro:property name="base_len" value="0.2"/>
<xacro:property name="base_height" value="0.127"/>

<!--base_link call to inertia macro-->
<xacro:box_inertia m="${base_mass}" w="${base_width}" h="${base_height}" d="${base_len}"/>

<!--Macro for inertia-->
<xacro:macro name="box_inertia" params="m w h d">
    <inertial>
        <mass value="${m}"/>
        <inertia ixx="${m / 12.0 * (d*d + h*h)}" ixy="0.0" ixz="0.0" iyy="${m / 12.0 * (w*w + h*h)}" iyz="0.0" izz="${m / 12.0 * (w*w + d*d)}"/>
    </inertial>
</xacro:macro>
```





## Inertias

Info obtained from [wikipedia](https://en.wikipedia.org/wiki/List_of_moments_of_inertia)

### Box (chassis)

#### Parameters

- mass : $m$
- weight : $w$
- height : $h$
- diameter: $d$

#### Formula

$$i_{xx} = \frac{m}{12.0}(d^2 + h^2) $$

$$i_{xy} = 0.0$$

$$i_{xz} = 0.0 $$

$$i_{yy} = \frac{m}{12.0}(w^2 + h^2) $$

$$i_{yz} = 0.0 $$

$$i_{zz} = \frac{m}{12.0}(w^2 + d^2) $$



### Sphere (caster wheel)

#### Parameters

- mass : $m$
- radius : $r$

#### Formula

$$i_{xx} = \frac{2m(r²)}{5.0} $$

$$i_{xy} = 0.0$$

$$i_{xz} = 0.0 $$

$$i_{yy} = \frac{2m(r²)}{5.0} $$

$$i_{yz} = 0.0 $$

$$i_{zz} = \frac{2m(r²)}{5.0} $$



### Cylinder (left and right wheel)

#### Parameters

- mass : $m$
- radius : $r$
- height : $h$

#### Formula

$$i_{xx} = \frac{m(3r²+h²)}{12.0} $$

$$i_{xy} = 0.0$$

$$i_{xz} = 0.0 $$

$$i_{yy} = \frac{m(3r²+h²)}{12.0} $$

$$i_{yz} = 0.0 $$

$$i_{zz} = \frac{mr²}{2} $$



# Create URDF

Once the `xacro` files is created, it is possible to generate a `URDF` file as follows:

```
xacro rastreator.xacro > rastreator.urdf
```

In this case I am creating the `urdf` file from the `xacro` file using the python library `xacro`



# Create SDF

Once the `urdf` file is created, is it possible to generate a `SDF` file as follows:

```bash
gz sdf -p rastreator.urdf > ../model/rastreator_description/model.sdf
```

