"""
This code runs as to set up the script to run the VR in ratcave
"""

import pyglet
import pyglet.gl as gl
import ratcave as rc
from ratcave.resources import cube_shader, default_shader
from natnetclient import NatClient
from utils import get_screen, remove_image_lines_from_mtl, load_textured_mesh
import numpy as np
from numpy import linalg
from threading import Timer
import serial
import time
from time import strftime
from pyglet.window import key
import matplotlib.pyplot as plt
import math
import threading
import matplotlib as mpl
from mpl_toolkits.mplot3d import axes3d, Axes3D
import os
from pypixxlib.propixx import PROPixx




my_device = PROPixx()



# Experiment parameters:
fade=0. # 0-1 numbers only
animal_ID = 'FS1'
flat_shading_on = False
background_color = (0., 0., 0.)
cylinder_color = (0.+fade, .7+fade, 0.+fade)
arena_filename = 'assets/3D/beacon_scene.obj'  # note: make sure UV mapping and flipped normals in file
feeder_port = 'COM12'
actuator_port = 'COM7'
exposure_time = 1.5
time_in_cylinder = 1.5
circle = .15
rotation = 80
speed = .25
movement_collection_time = .01
save = True
cylinder_visible= True
height_end=0.01
height_start=0.821
my_device.setLampLED(False)
transition1 = 1
transition2 = transition1*2

#starting cylinder
xcylinder = 0.021457331
ycylinder = -0.5530283
alpha = 5



# Parameters never to change:
environment_color_filter = 1., 1., 1.

# arena floor max size - random distribution
x_diff = (0.37 + 0.22)-circle/2
xn = np.random.random() * x_diff - 0.37
z_diff = (0.59 + 1.02)-circle/2
zn = np.random.random() * z_diff - 0.59



def main():
    # getting positions of rigid bodies in real time
    client = NatClient()
    arena_rb = client.rigid_bodies['Arena']
    rat_rb = client.rigid_bodies['Rat']


    # connect to feeder
    feeder = serial.Serial(feeder_port, 9600)


    # connect to actuators
    actuator = serial.Serial(actuator_port, 9600)

    window = pyglet.window.Window(resizable=True, fullscreen=True, screen=get_screen(1))

    # Load Arena from a Blender .obj file
    remove_image_lines_from_mtl('assets/3D/beacon_scene.mtl')
    arena_reader = rc.WavefrontReader(arena_filename)
    arena = arena_reader.get_mesh("Arena", position=arena_rb.position)
    arena.uniforms['diffuse'] = environment_color_filter
    arena.rotation = arena.rotation.to_quaternion()  # needed for Motive tracker

    # Load the projector as a Ratcave camera, set light to its position
    projector = rc.Camera.from_pickle('assets/3D/projector.pkl')
    projector.position.x += .004
    projector.projection = rc.PerspectiveProjection(fov_y=40.5, aspect=1.777777778)
    light = rc.Light(position=projector.position)

    # Make Virtual Scene
    cube_mapping_projection = rc.PerspectiveProjection(aspect=1, fov_y=90, z_near=.001, z_far=10)
    rat_head_position = rat_rb.position
    rat_camera = rc.Camera(projection=cube_mapping_projection, position=rat_head_position)

    #plane = arena_reader.get_mesh("Plane")
    #plane = load_textured_mesh(arena_reader, 'Plane', 'sawdust.jpg')
    #plane.parent = arena
    #plane.uniforms['diffuse'] = cylinder_color
    #plane.uniforms['flat_shading'] = flat_shading_on

    #sky = arena_reader.get_mesh("Sky")
    #sky = load_textured_mesh(arena_reader, 'Sky', 'sky.png')
    #sky.parent = arena
    #sky.uniforms['diffuse'] = cylinder_color
    #sky.uniforms['flat_shading'] = flat_shading_on




    cylinder = load_textured_mesh(arena_reader, 'Cylinder')
    cylinder.parent = arena
    cylinder.uniforms['diffuse'] = cylinder_color
    cylinder.uniforms['flat_shading'] = flat_shading_on
    cylinder.position.y = -.22
    cylinder.position.z = xcylinder * np.cos(alpha) - ycylinder * np.sin(alpha)
    cylinder.position.x = xcylinder * np.sin(alpha) + ycylinder * np.cos(alpha)
    #cylinder.position.z = -0.5530283
    #cylinder.position.x = 0.021457331
    cylinder.time_in_cylinder = time_in_cylinder


    meshes = [cylinder]
    virtual_scene = rc.Scene(meshes=meshes, light=light, camera=rat_camera, bgColor= background_color)  # seetign aset virtual scene to be projected as the mesh of the arena
    virtual_scene.gl_states.states = virtual_scene.gl_states.states[:-1]
    virtual_scene.beg_of_recording = None

    # Make Cubemapping work on arena
    cube_texture = rc.TextureCube(width=4096, height=4096)  # usign cube mapping to import eh image on the texture of the arena
    framebuffer = rc.FBO(texture=cube_texture) ## creating a fr`amebuffer as the texture - in tut 4 it was the blue screen
    arena.textures.append(cube_texture)

    # Arena basics
    arena.last_move_action = 'b'
    arena.feed_counts = 0
    arena.in_hotspot_since = 0
    arena.in_reward_zone_since = 0
    arena.in_reward_zone_since2 = 0
    arena.in_refractory = False
    arena.cumulative_in = 0
    arena.timestamp = time.time()
    entry_duration_list = []
    entry_timestamp_list = []
    arena.cumulative_in2 = 0
    sham_entry_duration_list = []
    sham_entry_timestamp_list=[]
    Beacon_position_and_time = []
    entry_duration_graph=[]
    sham_entry_duration_graph=[]


    # starting description file
    time_stamp = strftime("%Y%m%d-%H%M%S")
    script_dir = os.path.dirname(os.path.dirname(__file__))
    results_dir = os.path.join(script_dir, 'Positions_%s/' % time_stamp)

    if not os.path.isdir(results_dir):
        os.makedirs(results_dir)



    computer_time=time.time()


    with open(results_dir +"metadata_%s.txt" % time_stamp, "a+") as f_meta:
        f_meta.write("Recording started on : %s  \r\n" % strftime("%Y-%m-%d %H:%M:%S"))
        f_meta.write ("Computer time was : %s  \r\n" % computer_time)
        f_meta.write ("exposure time : %s  \r\n" % exposure_time)
        f_meta.write ("time_in_cylinder : %s  \r\n" % time_in_cylinder)
        f_meta.write ("movement_collection_time : %s  \r\n" % movement_collection_time)
        f_meta.write ("animal_ID : %s  \r\n" % animal_ID)



    #To be able to use keyes

    keys = key.KeyStateHandler()
    window.push_handlers(keys)

    # updating the position of the arena in xyz and also in rotational perspective
    def make_cylinder_visible():
        cylinder.visible = cylinder_visible
        arena.in_refractory = False

    def make_cylinder_invisible():
        cylinder.visible = not cylinder_visible
        arena.in_refractory = False

    def in_hotspot():
        if not arena.in_hotspot_since:
            arena.in_hotspot_since = time.time()

    def start_recording():
        cylinder.visible = False
        cylinder.time_in_cylinder = 1000
        virtual_scene.beg_of_recording = time.time()
        print("Recording started on : %s \r\n" % strftime("%Y-%m-%d %H:%M:%S"))


        Timer(transition1, turn_lights_on).start()
        Timer(transition2, start_beacon).start()

    def turn_lights_on():
        my_device.setLampLED(True)
        return

    def turn_lights_off():
        my_device.setLampLED(False)
        return

    def start_beacon():
        make_cylinder_visible()
        cylinder.time_in_cylinder = 1.5


    #ploting 3d movement
    ratx = []
    raty = []
    ratz = []


 #Calculating distance
    def calculateDistance(x,y):
        travel=0
        for i in range(len(y)-1):
            dist = math.sqrt((x[0+i] - x[1+i])**2 + (y[0+i] - y[1+i])**2)
            travel+=dist

        return travel

   # Speed calculation
    def calculateSpeed(x,y,time):
        travel=0
        speed=[]
        for i in range(len(y)-1):
            dist = math.sqrt((x[0+i] - x[1+i])**2 + (y[0+i] - y[1+i])**2)/time
            speed.append(dist)

        return speed




    def update(dt):
        if virtual_scene.beg_of_recording is None:
            if height_end < rat_rb.position.y < height_start:
                start_recording()
            return

        """main update function: put any movement or tracking steps in here, because it will be run constantly!"""
        virtual_scene.camera.position.xyz = rat_rb.position
        arena.uniforms['playerPos'] = rat_rb.position
        arena.position, arena.rotation.xyzw = arena_rb.position, arena_rb.quaternion
        arena.position.y -= .02
        rat_position = rat_rb.position.x, rat_rb.position.z
        cylinder_position = cylinder.position_global[0], cylinder.position_global[2]
        sham_position = 0.026124984,0.21062018
        diff_position = np.array(rat_position) - np.array(cylinder_position)
        sham_diff_position = np.array(rat_position) - np.array(sham_position)
        distance = linalg.norm(diff_position)
        sham_distance = linalg.norm(sham_diff_position)








        #For Troubleshooting

        #print ("position x: %s" %cylinder.position_global[0])
        #print ("position y: %s" %cylinder.position_global[2])
        #print ("position x: %s" %rat_rb.position.x)
        #print ("position y: %s" %rat_rb.position.z)
        #print(time.time()-arena.in_reward_zone_since)


        # counting time in reward zone - anytime
        if distance < circle:

            if not arena.in_reward_zone_since:
                arena.in_reward_zone_since = time.time()

        else:
            if arena.in_reward_zone_since > 0:
                arena.cumulative_in += time.time() - arena.in_reward_zone_since
                entry_duration_list.append(time.time() - arena.in_reward_zone_since)
                entry_duration_list.append(time.time())
                entry_timestamp_list.append(time.time() - virtual_scene.beg_of_recording)
                entry_duration_graph.append((time.time() - arena.in_reward_zone_since))
                with open(results_dir + "beacon_entry_%s.txt" % time_stamp, "a+") as f_beacon_entry:
                    x, y, z = rat_rb.position
                    f_beacon_entry.write("%s %s %s %s %s %s %s\n" % (time.time(), x, y, z,cylinder.position.x,cylinder.position.z,time.time() - arena.in_reward_zone_since))


            arena.in_reward_zone_since = 0


           #SHAM

        if sham_distance < circle:
            if not arena.in_reward_zone_since2:
                arena.in_reward_zone_since2 = time.time()

        else:
            if arena.in_reward_zone_since2 > 0:
                arena.cumulative_in2 += time.time() - arena.in_reward_zone_since2

                sham_entry_duration_list.append(time.time() - arena.in_reward_zone_since2)
                sham_entry_duration_list.append(time.time())
                sham_entry_timestamp_list.append(time.time() - virtual_scene.beg_of_recording)
                sham_entry_duration_graph.append((time.time() - arena.in_reward_zone_since2))
#TOFIX
            #with open("sham_beacon_entry %s.txt" % time_stamp, "a+") as f_sham_beacon_entry:
             #       x, y, z = rat_rb.position
              #      f_sham_beacon_entry.write("%s %s %s %s %s %s %s\n" % (time.time(), x, y, z,sham_position[0],sham_position[1],time.time()- arena.in_reward_zone_since2))

            arena.in_reward_zone_since2 = 0

        if distance < circle and not arena.in_refractory:
            in_hotspot()

            if time.time() - arena.in_hotspot_since > cylinder.time_in_cylinder and time.time() - arena.in_reward_zone_since < cylinder.time_in_cylinder +.02:
                cylinder.visible = False
                feeder.write('f')
                arena.feed_counts += 1
                print("Feed counts: %s at %s total %0.2f " % (arena.feed_counts, strftime("%H:%M:%S"), (time.time() - arena.in_reward_zone_since) + arena.cumulative_in))

                with open(results_dir +"metadata_%s.txt" % time_stamp, "a+") as f_meta:
                    f_meta.write("Pellet # %d dispensed on %s real time: %s \r\n" % (arena.feed_counts, strftime("%H:%M:%S"),time.time()))

                with open(results_dir + "beacons_%s.txt" % time_stamp, "a+") as f_beacon:
                    x, y, z = rat_rb.position
                    f_beacon.write("%s %s %s %s %s %s\n" % (time.time(), x, y, z,cylinder.position.x,cylinder.position.z))


                if ((arena.feed_counts) % 4) == 0:
                    zn = np.random.random() * z_diff - (z_diff / 2.)
                    xn = np.random.random() * x_diff - (x_diff / 2.)

                    x = xn * np.cos(np.pi * alpha / 180.) - zn * np.sin(np.pi * alpha / 180.)
                    z = xn * np.sin(np.pi * alpha / 180.) + zn * np.cos(np.pi * alpha / 180.)
                    cylinder.position.xz = x, z
                    #cylinder.position.y = -.1
#TOFIX
                   # x = xn * np.cos(np.pi * alpha / 180.) - zn * np.sin(np.pi * alpha / 180.)
                    #z = xn * np.sin(np.pi * alpha / 180.) + zn * np.cos(np.pi * alpha / 180.)
                    #sham_position = x,z


                if ((arena.feed_counts) % 2) == 0:
                    #zn = np.random.random() * z_diff - (z_diff / 2.)
                    #xn = np.random.random() * x_diff - (x_diff / 2.)

                    #x = xn * np.cos(np.pi * alpha / 180.) - zn * np.sin(np.pi * alpha / 180.)
                    #z = xn * np.sin(np.pi * alpha / 180.) + zn * np.cos(np.pi * alpha / 180.)
                    #cylinder.position.xz = x, z
                    #cylinder.position.y = -.1

                    t1 = Timer(exposure_time, make_cylinder_visible)
                    turn_lights_on()
                    t1.start()
                    Beacon_position_and_time.append(cylinder.position.xz)
                    Beacon_position_and_time.append(time.time())

                else:
                    t1 = Timer(exposure_time, make_cylinder_invisible )
                    turn_lights_off()
                    t1.start()
                    Beacon_position_and_time.append(cylinder.position.xz)
                    Beacon_position_and_time.append(time.time())



                arena.in_refractory = True



        else:
            arena.in_hotspot_since = 0

        # keys handling
        if keys[key.LEFT]:
            cylinder.position.x -= speed*dt
        if keys[key.RIGHT]:
            cylinder.position.x += speed*dt
        if keys[key.UP]:
            cylinder.position.z -= speed*dt
        if keys[key.DOWN]:
            cylinder.position.z += speed*dt
        if keys[key.A]:
            cylinder.rotation.x +=rotation *dt
        if keys[key.D]:
            cylinder.rotation.z +=rotation *dt
        if keys[key.C]:
            cylinder.position.xz = 0.0,0.0
        if keys[key.V]:
            cylinder.visible = True
        if keys[key.I]:
            cylinder.visible = False


        # write position to the file

        time_since_last = time.time() - arena.timestamp
        if movement_collection_time < time_since_last:

            with open(results_dir + "position_%s.txt" % time_stamp, "a+") as f_pos:
                x, y, z = rat_rb.position
                f_pos.write("%s %s %s %s\n" % (time.time(), x, y, z))

                ratx.append(rat_rb.position.x)
                raty.append(rat_rb.position.z)
                ratz.append(rat_rb.position.y)


            arena.timestamp = time.time()

       # print(arena.position.xz)

    pyglet.clock.schedule(update)  # making it so that the app updates in real time


    @window.event
    def on_draw():
        # Render virtual scene onto cube texture
        with framebuffer:
            with cube_shader:
                virtual_scene.draw360_to_texture(cube_texture)

        # Render real scene onto screen
        gl.glColorMask(True, True, True, True)
        window.clear()

        with cube_shader:  # using cube shader to create the actuall 6 sided virtual cube which gets upated with position and angle of the camera/viewer
            rc.clear_color(255, 0, 0)
            # why is it here 39? e
            with projector, light:
                arena.draw()

    # anything in the closing message goes here|
    @window.event
    def on_close():



        #f_meta.write("Animal dispensed: %s pellets, spent %0.2f seconds in the reward zone and %0.2f in SHAM \r\n" % (arena.feed_counts,arena.cumulative_in,arena.cumulative_in2))
        #f_meta.write("Animal beacon stay histogram: %s \r\n\n" % (Beacon_position_and_time))
        #f_meta.write("Animal beacon stay histogram: %s \r\n\n" % (entry_duration_list))
        #f_meta.write("Animal SHAM beacon stay histogram: %s \r\n\n" % (sham_entry_duration_list))
        #f_meta.write("Animals speed: %s \r\n\n" % (calculateSpeed(ratx,raty,movement_collection_time)))

        print("Animal dispensed: %s pellets, spent %0.2f seconds in the reward zone and %0.2f in SHAM \r\n" % (arena.feed_counts,arena.cumulative_in,arena.cumulative_in2))
        print (entry_duration_list)
        print ("Animal traveled: %s meters" % (calculateDistance(ratx,raty)))

            #Transformations

        alpha = (5) * np.pi / 180
        ratxX = (np.asarray(ratx)) * (np.cos(alpha)) - (np.asarray(raty)) * np.sin(alpha)
        ratyY = (np.asarray(ratx)) * (np.sin(alpha)) + (np.asarray(raty)) * np.cos(alpha)


        #Plotting
        plt.style.use('ggplot')
        fig,ax = plt.subplots(1,2, figsize=(18, 9),sharey=True,sharex=True)
        fig.text(0.30, 0.8, 'Time in beacon: %0.0f '% arena.cumulative_in, bbox=dict(facecolor='green', alpha=.5),weight="bold")
        fig.text(0.75, 0.8, 'Time in SHAM beacon: %0.0f '% arena.cumulative_in2, bbox=dict(facecolor='cyan', alpha=.5),weight="bold")
        ax[0].hist(entry_duration_graph, bins = 20,color='olive')
        ax[0].set(xlabel='time (s)', ylabel='frequency',title='beacon stays')
        ax[0].set_yscale('log')
        ax[1].hist(sham_entry_duration_graph, bins = 20,color='teal')
        ax[1].set_yscale('log')
        ax[1].set(xlabel='time (s)', ylabel='frequency',title='SHAM beacon stays')

        fig.canvas.set_window_title('Beacon stays')
        fig.tight_layout()
        plt.show()

        fig2 = plt.figure(figsize=(18, 9))
        ax = fig2.gca(projection='3d')
        ax = fig2.add_subplot(1, 1, 1, projection='3d')
        ax.set(xlabel='x_position', ylabel='Y-position',zlabel = 'Height',title='beacon stays')
        ax.plot(ratxX, ratyY, ratz,)
        ax.view_init(-65, 70)

        plt.show()

        fig3,ax1 = plt.subplots(1,3, figsize=(18, 9))
        ax1[0].hist2d(ratxX, ratyY, bins=20,)
        ax1[0].set(xlabel='X', ylabel='Y',title='Movement histogram',)
        ax1[1].plot(calculateSpeed(ratxX,ratyY,movement_collection_time))
        ax1[1].set(xlabel='time', ylabel='speed',title='Velocity graph')
        ax1[2].hist(entry_timestamp_list,bins = 40,color='gold')
        ax1[2].set(xlabel='time point', ylabel='frequency',title='beacon entries')
        fig3.text(0.75, 0.8, 'Number of pellets: %0.0f '% arena.feed_counts, bbox=dict(facecolor='yellow', alpha=.5), weight="bold")
        fig3.text(0.45, 0.8, 'Distance traveled: %0.2f meters' % (calculateDistance(ratx,raty)), bbox=dict(facecolor='olive', alpha=.5),weight="bold")
        fig3.tight_layout()

        plt.show()

        #saving in subfile


        script_dir = os.path.dirname(os.path.dirname(__file__))
        results_dir = os.path.join(script_dir, 'Graphs_%s/' % time_stamp)

        if not os.path.isdir(results_dir):
            os.makedirs(results_dir)


        #To save or not?
        if save == True:

            fig.savefig(results_dir + "hist_%s"  % time_stamp)
            fig2.savefig(results_dir + "3D_%s"  % time_stamp)
            fig3.savefig(results_dir + "2Dhist_%s"  % time_stamp)
            #os.remove(" %s.txt" % strftime("%Y%m%d-%H%M%S"))




    pyglet.app.run()