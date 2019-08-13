"""
This code runs as to set up the script to run the VR in ratcave
"""

import pyglet
import pyglet.gl as gl
import ratcave as rc
from ratcave.resources import cube_shader, default_shader
from natnetclient import NatClient
from utils import get_screen, remove_image_lines_from_mtl
import numpy as np
from numpy import linalg
from threading import Timer
import serial
import time
from pyglet.window import key

# Experiment parameters:
fade=1 # 0-1 numbers only
flat_shading_on = True
background_color = (0., 0., 0.)
cylinder_color = (0.+fade, 0.+fade, 0.+fade)
arena_filename = 'assets/3D/beacon_scene.obj'  # note: make sure UV mapping and flipped normals in file
feeder_port = 'COM12'
actuator_port = 'COM7'
exposure_time = 7.0
time_in_cylinder = 1
circle = .15

# Parameters never to change:
environment_color_filter = 1., 1., 1.

# arena floor max size - random distribution
x_diff = (0.37 + 0.22)
x = np.random.random() * x_diff - 0.37
z_diff = (0.59 + 1.02)
z = np.random.random() * z_diff - 0.59


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
    cylinder = arena_reader.get_mesh("Cylinder")
    cylinder.parent = arena
    cylinder.uniforms['diffuse'] = cylinder_color
    cylinder.uniforms['flat_shading'] = flat_shading_on
    cylinder.position.y = -.01



    meshes = [cylinder]
    virtual_scene = rc.Scene(meshes=meshes, light=light, camera=rat_camera, bgColor= background_color)  # seetign aset virtual scene to be projected as the mesh of the arena
    virtual_scene.gl_states.states = virtual_scene.gl_states.states[:-1]

    # Make Cubemapping work on arena
    cube_texture = rc.TextureCube(width=4096, height=4096)  # usign cube mapping to import eh image on the texture of the arena
    framebuffer = rc.FBO(texture=cube_texture) ## creating a fr`amebuffer as the texture - in tut 4 it was the blue screen
    arena.textures.append(cube_texture)

    # last arena move
    arena.last_move_action = 'b'
    arena.feed_counts = 0
    arena.in_hotspot_since = 0
    arena.in_refractory = False

    keys = key.KeyStateHandler()
    window.push_handlers(keys)

    # updating the position of the arena in xyz and also in rotational perspective
    def make_cylinder_visible():
        cylinder.visible = True
        arena.in_refractory = False

    def in_hotspot():
        if not arena.in_hotspot_since:
            arena.in_hotspot_since = time.time()

    def update(dt):
        """main update function: put any movement or tracking steps in here, because it will be run constantly!"""
        virtual_scene.camera.position.xyz = rat_rb.position
        arena.uniforms['playerPos'] = rat_rb.position
        arena.position, arena.rotation.xyzw = arena_rb.position, arena_rb.quaternion
        arena.position.y -= .02

        rat_position = rat_rb.position.x, rat_rb.position.z
        cylinder_position = cylinder.position_global[0], cylinder.position_global[2]
        diff_position = np.array(rat_position) - np.array(cylinder_position)
        distance = linalg.norm(diff_position)
        #print ("position x: %s" %cylinder.position_global[1])
        #print ("position y: %s" %cylinder.position_global[2])

        if distance < circle and not arena.in_refractory:
            in_hotspot()

            if time.time() - arena.in_hotspot_since > time_in_cylinder:
                cylinder.visible = False
                feeder.write('f')
                arena.feed_counts += 1
                print("Feed counts: %s" % arena.feed_counts)
                #z = np.random.random() * z_diff - 0.59
                #x = np.random.random() * x_diff - 0.37
                #cylinder.position.xz = x, z
                #cylinder.position.y = -.1

                t1 = Timer(exposure_time, make_cylinder_visible)
                t1.start()

                arena.in_refractory = True

        else:
            arena.in_hotspot_since = 0

        if keys[key.LEFT]:
            cylinder.position.x -=.2*dt
        if keys[key.RIGHT]:
            cylinder.position.x +=.2*dt
        if keys[key.UP]:
            cylinder.position.z -=.2*dt
        if keys[key.DOWN]:
            cylinder.position.z +=.2*dt
        if keys[key.A]:
            cylinder.rotation.x +=80 *dt
        if keys[key.D]:
            cylinder.rotation.z +=80 *dt


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


    pyglet.app.run()