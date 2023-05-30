# Kogrob object detection
[//]: # (Image References)

[image1]: ./assets/vilag.png
[image2]: ./assets/frame.png
[image3]: ./assets/Detection.png
[image4]: ./assets/degree45.png
[image5]: ./assets/car.png
[image6]: ./assets/dummy.png

# Bevezetés
Feladatunk egy olyan Turtlebot 3 segítségével megvalósított projekt létrehozása volt, amelyben a Turtlebotot lehelyezzük egy általunk készített világba, ő végighalad a téren az általunk definiált pályás és mi a ráhelyezett kamera képén neurális háló segítségével azonosítjuk az útjába kerülő tárgyakat/embereket.
## A működés demonstrálása
## Tartalomjegyzék
1. [Szükséges telepítések](#szükséges-telepítések)
2. [Turtlebot 3 Submodule](#turtlebot-3-submodule)
3. [Az alkalmazott neurális háló](#az-alkalmazott-neurális-háló)
4. [A világ](#a-világ)
5. [Detektálás indítása](#detektálás-indítása)
6. [Automatikus mozgatás](#automatikus-mozgatás)
7. [Összefoglalás](#összefoglalás)
# Szükséges telepítések
Természetsen szükségünk van a [ROS Noetic](http://wiki.ros.org/noetic/Installation) `full desktop` verziójának telepítésére.
Emellett még kameraképre és Pythonra is szükségünk van a programot futtató gépre.
```bash
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-compressed-image-transport
sudo apt install python3-roslaunch
```
Et követően a [neurális háló](https://cv.gluon.ai/build/examples_detection/demo_ssd.html) megfelelő futtatásához szükségünk van még néhány parancssoros műveletre:
```bash
pip install mxnet
pip install gluoncv
pip install opencv-python
```
Majd húzzuk be ezt a repositoryt a Catkin Workspaceünk `src` mappájába.
>`git clone https://github.com/akos-barta/Kogrob_object_detection/tree/main`
Fontos megemlíteni, hogy a [turtlebot3](https://github.com/Mortharos/turtlebot3) repository submoduleként van hozzáadva a projekthez, ezért annak érdekében, hogy ez letöltésre kerüljön a futtató gépre, a parancssoron a `Kogrob_object_detection/object_detection` mappában a következő két parancsot kell egymás után lefuttatni:
```bash
git submodule init
git submodule update
```
Ezekután a turtlebot3 submodul is meg fog jelenni a könyvtárban.
# Turtlebot 3 Submodule
Néhány apró módosítást végeztünk az eredeti `turtlebot3` repositoryn, amelyeket a következőkben szeretnénk összefoglalni.
## Kamera hozzáadása
## Kamera hozzáadása
A kamera hozzáadása 2 fájlt érint, a `/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro` és a `/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.gazebo.xacro` fájlokat. Az előbbi a robot 3D modelljéhez adja hozzá a kameránk modelljét - egy kb. 8,6 fokkal megemelt piros kockát:
```xml
...
  <!-- Camera -->
  <joint type="fixed" name="camera_joint">
    <origin xyz="0.03 0 0.11" rpy="0 0.79 0"/>
    <child link="camera_link"/>
    <parent link="base_link"/>
    <axis xyz="0 1 0" />
  </joint>

  <link name='camera_link'>
    <pose>0 0 0 0 0 0</pose>
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <geometry>
        <box size=".01 .01 .01"/>
      </geometry>
    </collision>

    <visual name='camera_link_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".02 .02 .02"/>
      </geometry>
    </visual>

  </link>

  <gazebo reference="camera_link">
    <material>Gazebo/Red</material>
  </gazebo>

  <joint type="fixed" name="camera_optical_joint">
    <origin xyz="0 0 0" rpy="-1.5707 0 -1.5707"/>
    <child link="camera_link_optical"/>
    <parent link="camera_link"/>
  </joint>

  <link name="camera_link_optical">
  </link>
...
```

A másik fájl a kamera modelljéhez tartozó szimulált kamerát hozza létre:
```xml
...
  <!-- Camera -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera">
      <update_rate>30.0</update_rate>
      <visualize>false</visualize>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <!-- Noise is sampled independently per pixel on each frame.
               That pixel's noise value is added to each of its color
               channels, which at that point lie in the range [0,1]. -->
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>camera</cameraName>
        <imageTopicName>image</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link_optical</frameName>
        <hackBaseline>0.0</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>
...
```
# Az alkalmazott neurális háló
Az általunk alkalmazott neurális háló a `ssd_512_resnet50_v1_voc` névre hallgat. A hálóról bővebb információt a [Model Zoo](https://cv.gluon.ai/model_zoo/index.html) keresztül lehet megtudni.
A neurális háló működését az `object_detector.py` Python node valósítja meg.
```bash
net = model_zoo.get_model('ssd_512_resnet50_v1_voc', pretrained=True)
```
Parancs segítségével első futtatásra letölti a program a Model Zooból a szükséges súlyokat, ezért az első betöltés egy kicsit több időt fog igénybe venni. Ezt következően viszont ugyanezen parancs futásakor a már letöltött modelt fogja betölteni a program.
Számunkra a program legérdekesebb része az `AIprocess()` függvény.
```python
def AI_process(picture):
    # this is the object detector function
    # first of all we have to do some transformation on the image to be able to feed it to the network model
    image_rgb = cv2.cvtColor(picture, cv2.COLOR_BGR2RGB)
    pic = mx.nd.array(image_rgb)
    x, img = data.transforms.presets.ssd.transform_test(pic,short=512)
    class_IDs, scores, bounding_boxes = net(x)

    ax = utils.viz.plot_bbox(img, bounding_boxes[0], scores[0],
                             class_IDs[0], class_names=net.classes)
    
    ax.axis('off')
    plt.tight_layout()

    #the output of the model is a plot, so we have to save it as an image
    fig = ax.get_figure()
    fig.canvas.draw()
    img_data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    img_data = img_data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    pil_img = Image.fromarray(img_data)

    output_file = 'output.jpg'
    pil_img.save(output_file, 'JPEG')

    return output_file

```
Ahol a függvény a kamera éltal beérkezett képet először átkonvertálja olyan formátumba, amit a neurális háló elfogad, a neurális háló elkészíti a detektált objektumokhoz tartozó jelölő téglalapokat, majd egyesíti azt az eredeti képpel és a kimenetet pedig vissaadja a függvényt meghívó `processImage()` függvénynek, ahol visszatranszformáljuk a képet és megjelenítjük azt a `Detection` ablakban.
# A világ
```bash
roslaunch object_detection object_detection.launch
```
>Mindehhez futtassuk először egy külön ternimálban a `roscore` parancsot
parancs segítségével tudjuk futtatni az általunk készített világot.

![alt text][image1]

Maga a `object_detection.launch` fájlban behívjuk az `object_detection.world` fájlt, illetve hozzáadjuk a robotot az általunk kívánt pozícióban.
```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>

    <!--Robot arguments-->
    <arg name="model" default="burger" doc="model type [burger, waffle, waffle_pi]"/>
    <arg name="x_pos" default="0.0"/>
    <arg name="y_pos" default="-3.0"/>
    <arg name="z_pos" default="0.0"/>

    <!-- Start up simulated world -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find object_detection)/worlds/object_detection.world"/>
        <arg name="paused" value="false"/>
        <arg name="use_sim_time" value="true"/>
        <arg name="gui" value="true"/>
        <arg name="headless" value="false"/>
        <arg name="debug" value="false"/>
    </include>
     
    <!-- TurtleBot3 -->
    <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />
    <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-urdf -model turtlebot3 -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />
    
</launch>
```
>A világban legegyszerűbben a `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch` futtatásával van lehetőségünk mozogni.
# Detektálás indítása
Következő lépésben már futtathatjuk az `object_detector.py` nodeunkat egy külön terminálban, amely hatására felugrik két ablak az egyik tartamazza kameraképet, a másikon pedig a neurális háló által detektált objektumok lesznek megtalálhatók.
![alt text][image2]
![alt text][image3]   

Összegezve ezen négy parancs egyidejű futtatása szükséges kézzel történő mozgatás esetén:
```bash
roscore
roslaunch object_detection object_detection.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
rosrun object_detection object_detector.py
```
# Automatikus mozgatás
Lehetőségünk van a robotot automatikusan is mozgatni a `movement.py` node futtatásával egy külön terminálban. Ennek hatására a robot egy általunk definiált pályán fog végighaladni. Maga a node a fizikából jól ismert `s=v*t` összefüggés segítségével számolja, hogy meddig kell ahhoz mozognia, hogy 1 méternyi utat megtegyen. A program. A `Ctrl+C` billenytű kombináció htására pedig a robot megáll és ezzel együtt a node is leállításra kerül.
Viszont észrevettünk egy olyan hibát, ami folytán amennyiben kiadjuk a robotnak a gazebos környezetben, hogy végezzen egyenes vonalú egyenletes mozgást, ő mindig le fog térni a pályáról és egy bizonyos idő elteltével beáll a rácsvonalakra átlós irányú (45°-kal elforgatott) pályára. Viszont amennyiben alapból ilyen irányban indítjuk el a robotot, akkor képes egyenes vonalú egyenletes mozgást végezni. éppen ezért létrehoztunk egy `object_detection_45degree.world` fájlt, amely 45 fokban van elforgatva.
![alt text][image4]
Az ehhez tartozó külön Launch fájl pedig az `object_detection_45degree.launch`
Összesítve ebben az esetben is négy különböző parancs egyidejű, külön terminálban történő futtatására van szükség. Ezek:
```bash
roscore
roslaunch object_detection object_detection_45degree.launch
rosrun object_detection object_detector.py
rosrun object_detection movement.py
```
# Összefoglalás
A tesztelések során azt tapasztaltuk, hogy a neurális háló viszonylag pontosan detektálta az útjába kerülő objektumokat. Természetes találkozhatunk anomáliákkal is a futtatás során. Példának okáért a kerekekkel rendelkező ágyat hajlamos a modell kocsiként felismerni. Ennek oka lehet az is, hogy a modell tanítása valós képpekkel történt, illetve abból is adódhat, hogy ilyen jellegő mintán nem lett betanítva.  

![alt text][image5]   

Kísérlet jelleggel pedig elhelyeztünk az egyik szobában egy fehér dummyt, ugyanis kíváncsiak voltunk, hogy vajon őt emberként ismeri-e fel. A tapasztalataink azok voltak, hogy ebben az esetben a model nem ismerte fel egyáltalán semmit a szobában, amit helyes eredményként konstatáltunk.  

![alt text][image6]  

Természetesen fejlesztési lehetőségként meg lehet említeni, hogy esetleg kipróbálhatnánk több különböző modellt ugyanezen feladat elvégzésére, illetve akár saját modellt is alkothatnánk, amely kifejezetten a szimulált környezetben lenne betanítva.
## Fejlesztők
```bash
Barta Ákos
Hanák Zoltán
Horváth Marcell
```
