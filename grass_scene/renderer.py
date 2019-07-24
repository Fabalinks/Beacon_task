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

# Experiment parameters:
flat_shading_on = True
background_color = (1., 0., 0.)
cylinder_color = (0., 1., 1.)
arena_filename = 'assets/3D/beacon_scene.obj'  # note: make sure UV mapping and flipped normals in file

# Parameters never to change:
environment_color_filter = 1., 1., 1.


def main():
    # getting positions of rigid bodies in real time
    client = NatClient()
    arena_rb = client.rigid_bodies['Arena']
    rat_rb = client.rigid_bodies['Rat']

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


    meshes = [cylinder]
    virtual_scene = rc.Scene(meshes=meshes, light=light, camera=rat_camera, bgColor= background_color)  # seetign aset virtual scene to be projected as the mesh of the arena
    virtual_scene.gl_states.states = virtual_scene.gl_states.states[:-1]

    # Make Cubemapping work on arena
    cube_texture = rc.TextureCube(width=4096, height=4096)  # usign cube mapping to import eh image on the texture of the arena
    framebuffer = rc.FBO(texture=cube_texture) ## creating a fr`amebuffer as the texture - in tut 4 it was the blue screen
    arena.textures.append(cube_texture)

    label = pyglet.text.Label(text='', font_size=50)

    # updating the posiotn of the arena in xyz and also in rotational perspective
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
        print(distance)
        label.text = str(distance)

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
        label.draw()
    pyglet.app.run()