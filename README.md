# ROS-AGV

para instalar ROS se recomienda usar una maquina virtual en windows. Siempre es mejor tener una particion de linux pero resulta mas facil y rapido usar la maquina virtual.

El procedimiento es el siguiente:
1. Descargar e instalar VMware Workstation 16 Player [aqui](https://www.vmware.com/co/products/workstation-player/workstation-player-evaluation.html). el software es gratuito para fines no comerciales
2. Descargar Ubuntu Mate 20.04.1 [aqui](https://ubuntu-mate.org/download/amd64/focal/).
3. intalar Ubuntu Mate en el VMware.
4. En la nueva instalacion de ubuntu, se debe instalar ROS abriendo una terminal y siguiendo los pasos de la guia de instalacion de ROS [aqui](http://wiki.ros.org/Installation/Ubuntu). A continuacion se resume el proceso de instalacion:

Abra una nueva terminal y copie los siguientes comandos en orden.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Los siguientes comando puede tardes bastante
```
sudo apt update
```

```
sudo apt install ros-noetic-desktop-full
```
Para trabajar con ROS, se debe correr el archivo setup.bash de ROS cada que se abre una terminal. Para que esto se haga de forma automatica cada que se abra una nueva terminal, se deben correr los siguientes comandos en la terminal:
* ``` echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc ```
* ``` source ~/.bashrc ```

Pruebe la instalacion iniciando al ambiente de simualacion **Gazebo**, en la terminar escriba ***gazebo*** y presiones enter. Si la aplicacion abre y se mantiene, todo quedo bien. Si al aplicacion se cierra y muestra un error en consola, ejecute el siguiente comando:
```
echo "export SVGA_VGPU10=0" >> ~/.profile
```

Para facilitar el trabajo con ROS en multiples terminales, se recomienda instalar el emulador de terminal "terminator":
```
sudo apt-get update
sudo apt-get install terminator
```
# capbot_tutorial

## Configurar el Workspace 

```
$ mkdir capbot_ws
$ cd capbot_ws
$ mkdir src
$ catkin_make
$ . ~/capbot_ws/devel/setup.bash
```
##Preparar el Modelo del robot

Crear paquete con dependencia de urdf (para visualizar el modelo desde ros)
```
$ cd src
$ catkin_create_pkg capbot_description urdf
$ cd capbot_description
```
Dentro de la carpeta ***capbot_description*** que se acaba de crear, se deben poner las carpetas ***urdf*** y ***meshes*** que se pueden descargar[aqui](https://javerianacaliedu-my.sharepoint.com/:f:/g/personal/juandavid_contreras_javerianacali_edu_co/EmJIYJQKr6xPlEttRKYFlH4ByxAgbMaU-C1fkgPEM6wkOA?e=o5mWnd). En la carpeta urdf esta el archivo capbot.urdf con la descripcion de la conematica y dinamica del robot. En la carpeta meshes estan los archivos STL de los modelos 3D que componen el robot, estos archivos se ensamblan segun la cinematica descrita en  capbot.urdf.

A continuacion se recomiendo hacer el analisis del modelo urdf utilizando un ***parser***. Para esto cree una carpeta **src/** dentro de capbot_description y en un editor de texto cree un archivo llamado **parser.cpp** y guardelo en la nueva carpeta **src/**. El contenido de **parser.cpp** es el siguiente:

```
#include <urdf/model.h>
#include "ros/ros.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "my_parser");
  if (argc != 2){
    ROS_ERROR("Need a urdf file as argument");
    return -1;
  }
  std::string urdf_file = argv[1];

  urdf::Model model;
  if (!model.initFile(urdf_file)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");
  return 0;
}
```
Para ejecutar el codigo, primero agregue las siguientes líneas al archivo CMakeList.txt de la carpeta capbot_description:
```
 add_executable(parser src/parser.cpp)
 target_link_libraries(parser ${catkin_LIBRARIES})
 ```

lo siguiente sera Construir (build) el paquete y ejecútelo, para esto en la terminal vaya a la carpeta capbot_ws usando cd o cd .. y pegue los siguientes comandos.
```
$ catkin_make
$ ./devel/lib/capbot_description/parser ./src/capbot_description/urdf/capbot.urdf
```
Si el analisis fue correcto se presentara un mensaje "Successfully parsed urdf file", de lo contrario se mostrara el error detectado.

Para visualizar el modelo URDF, podemos usar gazebo o rviz, lo recomendable es crear un archivo de lanzamiento "launch file".

Para crear nuestro primer launch file debemos crear una carpeta llamada launch en capbot_description y crear un archivos llamado gazebo.launch con el siguiente contenido:
```
<launch>
  <include
    file="$(find gazebo_ros)/launch/empty_world.launch" />
  <node
    name="spawn_model"
    pkg="gazebo_ros"
    type="spawn_model"
    args="-file $(find capbot_description)/urdf/capbot.urdf -urdf -model capbot"
    output="screen" />
</launch>
```
Recuerda guardar el documento.

los launch files son una herramienta muy util para iniciar varios nodos o funciones de ROS de forma integrada. en este archivo estamos abriendo un archivos de gazebo y entregando como argumento nuestro modelo capbot.urdf.

Para ejecutar el launch file debemos ejecutar los siguientes comandos.
```
$ cd ~/capbot_ws
$ source devel/setup.bash
$ roslaunch capbot_description gazebo.launch
```
Debera abrir el simulador Gazebo con el modelo del robot. Todavia no podemos mover el robot debido a que no tenemos ningun nodo que controle la velocidad de las ruedas.
___

## Crear un paquete de Gazebo para ROS

primero nos aseguramos de tener instaladas las dependencias para controlar gazebo desde ros
```
$ sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```
Crear el nuevo paquete con las dependencias necesarias
```
$ cd ~/capbot_ws/src
$ catkin_create_pkg capbot_gazebo gazebo_ros roscpp gazebo_msgs gazebo_plugins gazebo_ros_control
```
Crear las carpetas estandar para el almacenamiento de los archivos de la simulacion.

```
$ cd capbot_gazebo
$ mkdir launch
$ mkdir materials
$ mkdir models
$ mkdir worlds
$ mkdir src
```

Crear el archivo **capbot.launch** en la carpeta launch con el siguiente contenido:
```
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find capbot_gazebo)/worlds/capbot.world"/>
  </include>
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-file $(find capbot_description)/urdf/capbot.urdf -urdf -z 1 -model capbot" />
</launch>
```
Este archivo permite lanzar la simulacion en gazebo cargando automaticamente el entorno y el robot.
se puede notar que el argumento "world_name" hace referencia a un archivo **capbot.world** que no a sido creado y el nodo "spawn_urdf" hace referencia al archivo **capbot.urdf** creado en el punto anterior.

Para crear el entorno tipo ***world*** para la simulacion, se debe abrir gazebo y agregar los elementos que correspondan, finalizar guardando el modelo como capbot.world en la carpeta correspondiente. para abrir gazebo solo se debe escribir ***gazebo*** en la consola y explorar las funciones de agregar objetos y crear geometrias para poblar la simulacion.

Para iniciar la simulacion desde el archivo capbot.launch se utiliza la funcion roslaunch asi:
```
$ source devel/setup.bash
$ roslaunch capbot_gazebo capbot.launch
```
nota: el comando source devel/setup.bash debe usarse cada que se crean nuevos paquetes justo despues e usar catkin make para que los nuevos paquetes puedan usarse desde la terminal.
Este comando abrirá gazebo cargando el entorno y el robot pero no permitira mover el robot desde ROS (todavia), de echo, no es nada diferente al archivo ***gazebo.launch*** que usamos anteriormente, pero iremos agregando mas funciones.

## Configurar el robot para ser simulado en gazebo
El modelo del robot que es interpretado por gazebo es el archivo capbot.urdf, en este archivo se debera incluir toda la informacion y funciones (que llamaremos plugins) que permiten a gazebo interactuar con el robot para producir el movimiento o agregar sensores (simulados).

primero entendamos que contiene un archivo de descripcion de robot tipo URDF.

### Archivos URDF
URDF es un lenguaje de representacion de robot, algo similar a una matriz DH pero con una jerarquia, propiedades fisicas y modelos 3D, es decir, permite representar la cinematica, dinamica y geometria del robot. La forma de representar toda esta informacion es por medio de un archivo XML, asi que para entender urdf se debe entender algo de xml.

EL elemento principal del archivo urdf es el elemento robot con el unico atributo name, el cual contiene  pricipalmente elementos link y joint que se relacionan para formar el robot.
```
<robot name="robotname">
  <link> ... </link>
  <link> ... </link>
  <link> ... </link>
  <joint> .... </joint>
  <joint> .... </joint>
  <joint> .... </joint>
</robot>
```
adicionalmente, el elemento robot puede contener elementos como los plugins de gazebo para complementar el modelo.

los elementos tipo link describen los eslabones del robot, para un brazo robotico serian los eslabones que componen la cadena cinematica, para un robot movil seran las ruedas y el cuerpo del robot. Los elementos link tienen un atributo name por el que se le identifica, los subelementos que describen los link son:
* inertial: difine las propiedades inerciales del eslabon, contiene los siguientes elementos:
  - origin: posicion y rotacion del origen del eslabon
  - mass: valor de la masa en kg
  - inertia: momento de inercia con respecto al origen
* visual: propiedades visuales del eslabon, contiene los siguientes elementos:
  - origin: origen visual del elemento
  - geometry: describe el modelo 3D del elemento, nomalmente contiene un elementos mesh que hace referencia a un archivo STL (modelo 3D).
  - material: propieades visuales del eslabon como el color en formato rgba
* collision: propiedades para detectar colisiones a partir del espacio ocupado por el eslabon, normalmente se copian las mismas propiedades origin y geometry de elemento visual.
```
<link
    name="base_link">
    <inertial>
      <origin
        xyz="0.019462 0.00056937 0.12444"
        rpy="0 0 0" />
      <mass
        value="8.127" />
      <inertia
        ixx="0.083537"
        ixy="-9.6838E-07"
        ixz="-5.7145E-06"
        iyy="0.11259"
        iyz="2.8774E-06"
        izz="0.1919" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://capbot_description/meshes/base_link.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://capbot_description/meshes/base_link.STL" />
      </geometry>
    </collision>
  </link>
```
El elementos Joint define la articulaciones que uno dos eslabones. Este elemento tiene dos atributos:
* name: en nombre por el que se le identifica
* type: el tipo de articulacion que puede ser:
- revolute: una articulación de bisagra que gira a lo largo del eje y tiene un rango limitado especificado por los límites superior e inferior.
- continuous: una articulación en bisagra continua que gira en torno al eje y no tiene límites superior e inferior.
- prismatic: una junta deslizante que se mueve a lo largo del eje, y tiene un rango limitado especificado por los límites superior e inferior.
- fixed: Esto no es realmente una articulación, ya que no puede moverse. Todos los grados de libertad están bloqueados. Este tipo de unión no requiere ejes.
- floating: Esta articulación permite el movimiento de los 6 grados de libertad.
- planar: Esta articulación permite el movimiento en un plano perpendicular al eje.

Adicionalmente, el elemento joint contiene otros elementos que describen el movimiento y jerarquia:
* origin: posicion y rotacion de la articulacion con respecto al origin del eslabon padre
* parent: hace referencia al eslabon que precede a la articulacion, o el eslabon al que esta vinculado. esta referencia se hace en el atributo link dando en nombre del eslabon.
* child: hace referencia al eslabon que se movera por la articulacion. esta referencia se hace en el atributo link dando en nombre del eslabon.
* axis: difine por medio del atributo xyz cual es el eje sobre el que se hace el movimiento.
```
<joint
    name="L_joint"
    type="continuous">
    <origin
      xyz="0 0.238 0"
      rpy="1.5708 0 3.1416" />
    <parent
      link="base_link" />
    <child
      link="L_Link" />
    <axis
      xyz="0 0 1" />
  </joint>
```
el modelo urdf del capbot fue creado de forma automatica utilizando el complemento urdf para solidworks.

## gazebo plugins
Para conectar el modelo urdf de forma dinamica con gazebo, se deben incluir los plugins de gazebo en el archivo urdf, esto es simplemente agregar otros elementos.

Un listado completo de los plugins se puede encontrar en este [link](http://gazebosim.org/tutorials?tut=ros_gzplugins).


El primer plugin que debemos agregar al modelo de nuestro robot es el ***differential_drive_controller***, el cual nos permite controlar el movimiento del capbot. lo que hace este plugin es crear dentro de gazebo un controlador para robots diferenciales (como el capbot), este controlador recibe mensajes tipo Twist y los convierte en las correspondientes velocidades de rotacion en las ruedas. Para hacer esta conversion, el plugin necesita conocer la gemetria del robot (diamtro de las ruedas, distancia entre estas), tambien conocer propiedades cinematicas y dinamica como la aceletacion y el torque de las ruedas para representar de forma realista los motores. Finalmente, el plugin requiere informacion para comunicarse con ROS como el nombre de los topics y que informacion debe publicar o no. 

Todas las propiedades anteriores se agregan pegando el siguiente codigo dentro de elementos robot del archivo urdf:
```
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">

    <!-- Taza de actualizacion en Hz -->
    <updateRate>20</updateRate>
    <!-- nombre de la articulacion izquierda -->
    <leftJoint>L_joint</leftJoint>
    <!-- nombre de la articulacion derecha -->
    <rightJoint>R_joint</rightJoint>
    <!-- distancia entre las ruedas en metros -->
    <wheelSeparation>0.486</wheelSeparation>
    <!-- diametro de las ruedas en metros -->
    <wheelDiameter>0.112</wheelDiameter>
    <!-- acelaracion de las ruedas en rad/s^2-->
    <wheelAcceleration>5.0</wheelAcceleration>
    <!-- torque maximo producido por las ruedas en Nm -->
    <wheelTorque>3</wheelTorque>
    <!-- Topic en el que se recibiran los mensajes geometry_msgs/Twist  -->
    <commandTopic>cmd_vel</commandTopic>
    <!-- Topic en elque se publicaran los mensajes nav_msgs/Odometry que contienen la odometria -->
    <odometryTopic>odom</odometryTopic>
    <!-- frame de referencia para la odometria -->
    <odometryFrame>odom</odometryFrame>
    <!-- frame del robot desde el que se calcula la odometria -->
    <robotBaseFrame>base_link</robotBaseFrame>
    <!-- origen de la odometroa, 0  ENCODER, 1  WORLD -->
    <odometrySource>world</odometrySource>
    <publishTf>1</publishTf>
	<publishOdomTF>true</publishOdomTF>
    <rosDebugLevel>na</rosDebugLevel>
    <!-- true tpara publicar los  "transforms" para las ruedas, defaults  false -->
    <publishWheelTF>false</publishWheelTF>
    <!-- true para publicar "transforms" para la odometria, defaults  true -->
    <publishOdom>true</publishOdom>
    <!-- publicar mensaje sensor_msgs/JointState en el topic /joint_states, defaults  false -->
    <publishWheelJointState>false</publishWheelJointState>
    <!-- Strue tpara ivnertir las ruedas, defaults true -->
    <legacyMode>false</legacyMode>
  </plugin>
</gazebo>
```

Preste atencion a la descripcion de cada elemento en los comentarios del codigo. Si algun cambio se realiza en el robot, debe actualizarse el codigo para que corresponda.

El siguiente plugin a utilizar sera el de agregar un sensor tipo Lidar (radar de luz) que le permite al robot conocer la distacia de cualquier objeto a su alrededor. para esto utilizamos el plugin ***gazebo_ros_head_rplidar_controller*** agregando el siguiente codigo dentro del elemento robot del archivo urdf.
```
<!-- referencia vincula el sensor al link llamado "laser" del modelo urdf -->
<!-- se podria vincular tambien al link "base_link" pero corriendo la posicion -->
<gazebo reference="laser">
    <sensor type="ray" name="head_rplidar_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>40</update_rate>
      <ray>
        <scan>
          <horizontal>
            <!-- numero de muestras en los 360 grados -->
            <samples>360</samples>
            <!-- resolucion en grados -->
            <resolution>1</resolution>
            <min_angle>-3.14159265</min_angle>
            <max_angle>3.14159265</max_angle>
          </horizontal>
        </scan>
        <range>
          <!-- minima distacia desde la que empieza a medir -->
          <min>0.3</min>
          <!-- maxima distacia que puede medir -->
          <max>8.0</max>
          <!-- resolucion en metros -->
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <!-- conexion al plugin -->
      <plugin name="gazebo_ros_head_rplidar_controller" filename="libgazebo_ros_laser.so">
        <!-- nombre del topic en el que se publicaran los mensajes sensor_msgs/LaserScan -->
        <topicName>scan</topicName>
        <frameName>laser</frameName>
      </plugin>
    </sensor>
</gazebo>
```
preste atencio a los comentarios en cada linea para entender los elementos que parametrizan al plugin.

**actividad: agregar nuevos plugins para cambiar el color de los eslabones**

# Crear transformaciones
Una de los puntos mas confusos de ROS son las transformaciones. Estos son mensajes de tipo tf2_msgs/TFMessage que contienen la posicion de un sistema de referencia con respecto a otro.

Una descripcion detallada de las transformaciones puede encontrarse en este [tutorial](http://wiki.ros.org/tf).

Para nuestro robot es necesario publicar una transformacion (broadcasting a transform) que envie constantemente informacion sobre la posicion del Lidar con respecto a la base del robot. una buena explicacion de porque se requiere esta transformacion se encuentra [aqui](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF). Un resumen y adaptacion de esa guia se presenta a continuacion:

Primero navegamos en la terminal hasta la carpeta captbot_ws/src y creamos un nuevo paquete llamado ***capbot_setup_tf***.
```
$ catkin_create_pkg capbot_setup_tf roscpp tf geometry_msgs
```
Ahora vamos a crear un nodo, (hablaremos mas de la creacion de nodos en el siguiente tutorial), para esto entramos a la carpeta ***capbot_setup_tf*** que se acaba de crear y en ella creamos la carpeta ***src***.

```
$ cd capbot_setup_tf
$ mkdir src
```
en la nueva carpeta ***capbot_setup_tf/src*** creamos un nuevo archivo, lo llamamos ***tf_broadcaster.cpp***, abrimos y pegamos el siguiente codigo:

```
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//enviar la informacion de tf del robot
int main(int argc, char** argv){
  ros::init(argc, argv, "capbot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  ROS_INFO("running");
  while(n.ok()){
  //Vector3(0.0, 0.0, 0.3) indica que el lidar esta 30 cm sobre "base_link"
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.3)),
        ros::Time::now(),"base_link", "laser"));
    r.sleep();
  }
}
```


Para poder ejecutar este codigo desde ROS, debemos compilar usando catkin_make. Primero abrimos el archivo CMakeLists que esta en la carpeta ***capbot_setup_tf*** y agregamos lo siguiente al final de archivo y guardamos:
```
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
```
Esto le indica a la funcion catkin_make que debe compilar el archivo tf_broadcaster.cpp. 

Ahora ejecutamos:

```
$ cd ~/capbot_ws/
$ catkin_make
```

Al final nos deberia aparecer el mensaje **[100%] Built target tf_broadcaster** indicando que compilo correctamente.

# Tutorial 2: Controlando el movimiento del robot
En este tutorial utilizaremos diferentes estrategias para controlar manual y automaticamente el movimiento de robot. primero se presentan los conceptos fundamentales de la comunicacion entre nodos de ROS.


## Navegar por el sistema de archivos ROS

Los paquetes son la unidad de organización principales de ROS. Cada paquete puede contener bibliotecas, ejecutables, scripts u otros elementos. Los paquetes están asociados a un manifiesto (package.xml) el cual contiene descripción del paquete y sirve para definir dependencias entre paquetes y para capturar meta información sobre el paquete como versión, mantenedor, licencia, etc.

En un proyecto complejo se pueden usar muchos paquetes lo que hace que navegar usando comandos de consola como `cd` o `ls` sea muy difícil. Por esta razón, **ROS** tiene su propio sistema de navegación basado en comandos pero enfocado a encontrar paquetes sin tener que conocer o especificar la ruta.

Intente los siguientes comando:

```
$ rospack find rospy

$ roscd roscpp

$ rosls rospy
```

- El primer comando `rospack find` devuelve la ruta del paquete solo con su nombre.

- El comando `roscd` permite entrar al directorio (carpeta) del paquete solo por su nombre. También funciona con subcarpetas.

- El comando `rosls` devuelve la lista de carpetas y archivos que contiene un paquete.

Los paquete _roscpp_ y _rospy_ permiten correr funciones de **ROS** desde C++ y python respectivamente; se verán mas adelante.

Cierre la terminal y abra una nueva para el siguiente paso.

## Autocompletado por medio de TAP

Una función muy útil de **ROS** para hacer mas rápido el uso de la terminal de comandos, es el autocompletado por medio de la tecla Tab. Con esto, solo es necesario escribir las primeras letras de cada nombre y presionar Tab para que se complete el resto o se muestren las opciones cuando hay mas de una.

En la nueva terminal escriba los siguiente pero presionando Tab **dos** veces al final.

```

$ rosls rosc

```

Vera que se muestran todos los paquetes que empiezan por `rosc`, ahora escriba lo siguiente y presiones una vez Tab.

```

$ rosls roscpp_s

```

Vera que se completa `rosls roscpp_serialization` que es la única opción para completar. (puede ser otra dependiendo de los paquetes instalados).

De esta forma se pueden completar complicadas rutas de archivos para ejecutar los comandos correspondiente.

## Nodos

Los nodos son las unidades basicas de ROS, la arquitectura de ROS consiste en nodos que se comunican entre ellos, los paquetes son agrupaciones de nodos.

Conceptos básicos de la comunicación en ROS:

- **Nodos (node):** un nodo es un ejecutable que utiliza ROS para comunicarse con otros nodos.

- **Mensajes (messages):** Tipo de datos en ROS utilizado al suscribirse o publicar en un tema.

- **Temas (topic):** Los nodos pueden publicar mensajes en un tema, así como suscribirse a un tema para recibir mensajes.

- **Maestro (master):** Servicio encargado del registro de nombres en ROS (es decir, ayuda a los nodos a encontrarse)

- **rosout**: Equivalente ROS de stdout / stderr (funciones de terminal de linux)

- **roscore:** Master + rosout + servidor de parámetros (el servidor de parámetros se presentará más adelante)

Un nodo es un archivo ejecutable dentro de un paquete ROS. Los nodos ROS utilizan _rospy_ o _roscpp_ para comunicarse con otros nodos. Los nodos pueden publicar o suscribirse a un tema. Los nodos también pueden proporcionar o usar un Servicio.

### Roscore

Para utilizar la infraestructura de comunicación de ROS es necesario que _roscore_ este ejecutándose, esto implica tener una terminal dedicada donde se ejecuta el siguiente comando:

```

$ roscore

```

Esta terminal no se debe cerrar por lo que debe abrir otras terminales para ejecutar otros comandos. **Cuando se ejecuta gazebo desde el launch file, roscore corre internamente, asi que no es necesario llamarlo de nuevo mientras gazebo este activo**.

### Rosnode

_Rosnode_ es una función de ROS que permite obtener información de los nodos que están registrados en _rosmaster_. Roscore debe estar corriendo para poder usar rosnode.

Veamos que puede hacer rosnode, ejecuta en una nueva terminal:

```
rosnode list
```

Esto devuelve todos los nodos que se estén corriendo. En nuestro caso solo debería mostrar `/rosout` que es un nodo que siempre se esta ejecutando para manejar la comunicación. (si sale error en comunicacion es porque no estas ejecutando roscore en otra terminal).

Otro comando es _rosnode info_ el cual responde con la información del nodo solicitado. Esta información debe ser escrita por el desarrollador.

Ejecute:

```
rosnode info /rosout
```

Ahora veremos como activar mas nodos manualmente, pero primero vamos a iniciar la simulación de un robot para analizar la comunicación entre nodos.

Cierre todas las terminales y siga los siguientes pasos:

1. En la carpeta capbot_ws ejecute ```catkin_make``` para compilar todos los nodos y ejecute ```source devel/setup.bash```.

2. Ejecute gazebo con ```roslaunch capbot_gazebo capbot.launch```

3. Ejecute el tf_broadcaster con ```rosrun capbot_setup_tf tf_broadcaster```


La simulación puede tardar varios segundos o minutos en iniciar. Una vez cargado todo el ambiente de simulación, ejecute nuevamente `rosnode list` para ver todos los nodos que ahora se están ejecutando.

Aparecen los siguientes nodos.

```

/gazebo

/gmapping_node

/joint_state_controller_spawner

/move_base

/robot_state_publisher

/rosout

/rviz

```

Estos nodos iniciaron automáticamente, pero otros nodos se pueden iniciar desde la terminal usando `rosrun`, veremos mas adelante como usar este comando con los nodos que creamos.

## Comunicación por medio de topics

ROS permite dos tipos de comunicaciones entre nodos, _Topics_ y _Services_, vamos a enfocarnos primero en los topics, los servicios los estudiaremos a continuación.

Los _Topics_ permiten una comunicación tipo PUB/SUB (publicación - suscripción) en la cual un nodo crea un tema (topic) y otros nodos se suscriben a el, cada que se publique un mensaje en el topic, todos los nodos suscritos pueden leerlo.

El comando base para trabajar con _Topics_ es **rostopic**.
Un primer uso de este comando es la función de ayuda para leer otras funciones y comandos, en una nueva terminal escriba lo siguiente:

```

$ rostopic -h

```

Veremos con detalle algunos de estos comandos.

### rostopic echo

`rostopic echo` permite ver en la terminal los mensajes publicados en un tema.

Para conocer los temas que están abiertos podemos escribir el siguiente comando:

```

$ rostopic list

```

Esto despliega una lista de los temas abiertos, ahora vamos a leer el tema _/scan_ el cual es usado por el robot para publicar constantemente los valores leídos por el sensor laser de 360 grados.

```

$ rostopic echo /scan

```

No te alarmes!, aparecerán muchas lineas de información, es normal solo tienes que presionar _Ctrl + Z_ para dejar de leer.

Esto nos muestra un ejemplo de un mensaje publicado constantemente en un tema. Para usar esa información es necesario crear una suscripción, eso lo haremos mas adelante.

### Tipos de mensaje

Los temas transmiten mensajes, un mensaje tiene una estructura de datos compleja que puede ir desde un valor único hasta un conjunto de datos de diferentes tipos incluidos arreglos y listas.

Es necesario conocer el tipo de mensaje asociado a un tema para poder usarlo, ya sea para interpretar los datos y usarlos en una aplicación o para poder enviar un mensaje con la estructura correcta.

Para conocer el tipo de mensaje asociado a un tema, se usa el comando `rostopic type [topic]`. Podemos usarlo con el tema `/scan` así:

```

$ rostopic type /scan

# sensor_msgs/LaserScan

```

Ahora podemos usar `rosmsg show [message topic]` para conocer la estructura detallada del mensaje así:

```

$ rosmsg show sensor_msgs/LaserScan

```

Respuesta:

```
std_msgs/Header header
uint32 seq
time stamp
string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```

Estos muestra que hay varios datos de los cuales nos puede interesar que el dato `ranges` que es un arreglo de datos tipo float32, este dato contiene los valores leídos por el sensor desde _angle_min_ a _angle_max_ aumentando lo indicado en _angle_increment_.

Como otro ejemplo, podemos revisar la estructura del tema _/cmd_vel_, este tema es usado para enviar mensajes de movimiento al robot.

```
$ rostopic type /cmd_vel
```

Respuesta:

```
geometry_msgs/Twist
```

Ahora buscamos la estructura del mensaje:

```
$ rosmsg show geometry_msgs/Twist
```

Respuesta:

```
geometry_msgs/Vector3 linear
float64 x
float64 y
float64 z
geometry_msgs/Vector3 angular
float64 x
float64 y
float64 z
```

Esto corresponde a un mensaje del tipo: '[x, y, z]' '[x, y, z]' donde el primer vector es la velocidad lineal y el segundo la velocidad de rotación.


## Control Manual: enviando mensajes desde la terminal.

el comando `rostopic pub` permite escribir un mensaje en un tema desde la terminal.

La sintaxis del comando es:

```

rostopic pub [topic] [msg_type] [args]

```

Donde,

- [topic] es el nombre del tema tal como aparece al usar `rostopic list`

- [msg_type] es el tipo de mensaje tal como aparece al usar `rostopic type`

- [args] es el contenido del mesaje que debe tener la misma estructura mostra al usar `rosmsg show`

Ahora usemos este comando para mover el robot. en la terminal escribe:

```

rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.6, 0.0, 0.0]' '[0.0, 0.0, 2.0]'

```

El atributo `-1` indica que envía un solo mensaje y termina la comunicación.

En el simulador podrá ver como el robot inicia su movimiento donde '[0.6, 0.0, 0.0]' indica una velocidad lineal en X y '[0.0, 0.0, 2.0]' una velocidad de rotación en Z. Ahora tome un tiempo para jugar un poco con el robot cambiando estos valores manualmente.

## Control Manual: usar un keyboard teleop
Una de las ventajas de ROS es poder usar nodos y paquetes que ya estan desarrollados e integrarlos facilmente en mi proyecto.
Para este ejemplo, usaremos el node teleop_twist_keyboard que nos permite mover el robot desde el teclado del computador.

Primero en una nueva terminal instalamos la libreria:
```
$ sudo apt-get install ros-noetic-teleop-twist-keyboard
```

A continuacion, ejecutamos el nodo
```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

En la terminal aparecen instrucciones de como usar las teclas para mover el robot.

Esta libreria tambien se puee modificar descargando el codigo fuente desde github y cambiando por ejemplo el nombre del topic en el que se publica el mansaje de movimiento, esto resulta util si se tienen varios robots en la simulacion y cada unos "escucha" en topics diferentes.

# Control automatico: crear un nodo que controle en funcion del sensor

para crear un nodo que controle el robot de forma automatica, primero debemos crear un paquete que contenga este nodo. para eso creamos el paquete capbot_driver con la dependencia rospy:
```
$ cd capbot_ws/src
$ catkin_create_pkg capbot_driver rospy
```
Ahora entramos a la carpeta capbot_driver y creamos la carpeta scripts. En esta carpeta se guardan los achivos de python.
```
$ cd capbot_drive
$ mkdir scripts
```

En esta carpeta debemos crear el archivo driver.py con el siguiente contenido.
```
#!/usr/bin/env python

import rospy #todo nodo de ROS en python debe importar rospy

from sensor_msgs.msg import LaserScan #importa los tipos de mensaje

from geometry_msgs.msg import Twist

# le indicamos a rospy el nombre del nodo.
rospy.init_node('evadir1') #con este nombre se registra en el rosmaster
#Creamos un objeto para publicar en el topic /cmd_vel, Twist es el tipo de mensaje
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#rospy.rate() ndica la frecuencia en Hz con la que se va a repetir el loop de control.
rate = rospy.Rate(2)
#crea un objeto para guardar el mensaje de tipo Twist
vel = Twist()
#Se da un valor inicial a la velocidad del robot y la distancia del sensor
vel.linear.x = 0.3
vel.angular.z = 0
d = 0.1
#publica a velocidad (se envia al robot)
pub.publish(vel)
#creamos la funcion que haga girar el robot cuando detecte un objeto
def turn(msg): #el unico argumento es el arreglo de lecturas del laser
d = msg.ranges[0] #optiene la lectura de laser en la primera posicion del arreglo
# d es la distancia detectada por el laser junto en frente del robot en m
if d < 0.7: #se hay un bjeto muy cerca, gira
vel.angular.z = 0.5
vel.linear.x = 0
else: #si no, avanza
vel.linear.x = 0.3
vel.angular.z = 0
pub.publish(vel) #actualiza la velocidad en el robot

"""
se crea una suscripcion al topic /scan (medidor laser de 360 grados.
el segundo argumento es el tipo de mensaje LaserScan (se importo)
el tercer argumento es la funcion que atiende el mensaje recibido "callback"
"""

rospy.Subscriber("/scan", LaserScan, turn)

#inicia la ejecucion periodica cada 0.5s segun se indique en rospy.rate
rospy.spin()
```
Tome un tiempo para entender el funcionamiento del script leyendo los comentarios.

Guardamos el archivo y lo volvemos ejecutable con el siguiente comando:
```
$ chmod -x driver.py
```
Ahora debemos agregar el archivo de python a el CMakeLists.txt de la carpeta capbot_driver agregan el siguiente codigo al final del archivo.
```
catkin_install_python(PROGRAMS
   scripts/micontrol.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 ```
 
La funcion catkin_install_python esta por defecto comentada en el CMakeLists, es una buena practica descomentar la funcion y agregar los atributos correspondientes para que el archivo quede mas ordenado.
 
 Finalmente debemos compilar (asi python no se compile) con catkin_make y ejecutar el nodo.
 ```
 $ cd ~/capbot_ws
 $ catkin_make
 $ source devel/setup.bash
 $ rosrun capbot_driver mydriver.py
 ```
 Podra ver en el simulador (si todo sale bien) que el robot se mueve y gira cuando detecta algun obstaculo.
 
 ## Actividad 1

Modifique el archivo mydriver.py para que el robot mejore la evasión de obstáculos utilizando un rango mayor del LaserScan y haciendo que gire mas rápido cuanto mas cerca este del obstáculo.
 
 # Navegacion 2D en ROS

El Navigation Stak es un componente de ros que permite a un robot movil ser dirigido a un obetivo de forma autonoma, evadiendo obstaculos y definiendo continuamente una trayectoria. Para usar el paquete de navegacion se debe previamente tener las siguientes funcionalidades configuradas:

*El robot debe publicar informacion de las coordendas del marco de los marcos de referencia usando mensajes de tipo ***tf***.
*El robot debe publicar informacion del sensor Lidar o una camara de profundidad en los mensajes ***sensor_msgs/LaserScan*** o ***sensor_msgs/PointCloud*** respectivamente.
*Un nodo debe estar publicando informacion de la odometria usando los mensaes ***tf*** y ***nav_msgs/Odometry***.
*EL robot debe moverse al recibir mensajes de tipo ***geometry_msgs/Twist***.

 




 

