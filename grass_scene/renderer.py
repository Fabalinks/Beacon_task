"""
This code runs as to set up the script to run the VR in ratcave
"""

import pyglet
import pyglet.gl as gl
import ratcave as rc
from ratcave.resources import cube_shader, default_shader, obj_primitives
from natnetclient import NatClient
import itertools
from utils import get_screen, load_textured_mesh, remove_image_lines_from_mtl


def main():
    #gettign positions of rigib bodies in real time
    client = NatClient()
    arena_rb = client.rigid_bodies['Arena']
    rat_rb = client.rigid_bodies['Rat']


    window = pyglet.window.Window(resizable=True, fullscreen=True, screen=get_screen(1))  # Opening the basic pyglet window


    # Load Arena
    remove_image_lines_from_mtl('assets/3D/beacon_scene.mtl')
    arena_filename = 'assets/3D/beacon_scene.obj'# we are taking an arena which has been opened in blender and rendered to 3D after scanning it does not have flipped normals
    arena_reader = rc.WavefrontReader(arena_filename)  # loading the mesh of the arena thought a wavefrontreader
    arena = arena_reader.get_mesh("Arena", position=arena_rb.position)  # making the wafrotn into mesh so we can extrude texture ont top of it.
    arena.uniforms['diffuse'] = 1., 1., 1.  # addign a white diffuse material to the arena
    arena.rotation = arena.rotation.to_quaternion() # we also need to get arena's rotation not just xyz so it can be tracked and moved if it gets bumped

    # Load the projector as a Ratcave camera, set light to its position
    projector = rc.Camera.from_pickle('assets/3D/projector.pkl')  # settign the pickle filled of the projector, which gives us the coordinates of where the projector is
    projector.position.x += .004
    projector.projection = rc.PerspectiveProjection(fov_y =40.5, aspect=1.777777778)
    light = rc.Light(position=projector.position)

    ## Make Virtual Scene ##
    rat_camera = rc.Camera(projection=rc.PerspectiveProjection(aspect=1, fov_y=90, z_near=.001, z_far=10), position=rat_rb.position)  # settign the camera to be on top of the rats head
    cylinder = arena_reader.get_mesh("Cylinder")
    cylinder.parent = arena
    cylinder.uniforms['diffuse'] = (0., 1., 0.)
    cylinder.uniforms['flat_shading'] = True

    meshes = [cylinder]
    virtual_scene = rc.Scene(meshes=meshes, light=light, camera=rat_camera, bgColor=(0, 0, 255))  # seetign aset virtual scene to be projected as the mesh of the arena
    virtual_scene.gl_states.states = virtual_scene.gl_states.states[:-1]


    ## Make Cubemapping work on arena
    cube_texture = rc.TextureCube(width=4096, height=4096)  # usign cube mapping to import eh image on the texture of the arena
    framebuffer = rc.FBO(texture=cube_texture) ## creating a fr`amebuffer as the texture - in tut 4 it was the blue screen
    arena.textures.append(cube_texture)


    # updating the posiotn of the arena in xyz and also in rotational perspective
    def update(dt):
        """main update function: put any movement or tracking steps in here, because it will be run constantly!"""
        virtual_scene.camera.position.xyz = rat_rb.position
        arena.uniforms['playerPos'] = rat_rb.position
        arena.position, arena.rotation.xyzw = arena_rb.position, arena_rb.quaternion
        arena.position.y -= .02

    pyglet.clock.schedule(update)  # making it so that the app updates in real time


    @window.event
    def on_draw():

        ## Render virtual scene onto cube texture
        with framebuffer:
            with cube_shader:
                virtual_scene.draw360_to_texture(cube_texture)

        ## Render real scene onto screen
        gl.glColorMask(True, True, True, True)
        window.clear()
        with cube_shader:  # usign cube shader to create the actuall 6 sided virtual cube which gets upated with position and angle of the camera/viewer
            rc.clear_color(255, 0, 0)
              # why is it here 39? e
            with projector, light:
                arena.draw()

    # actually run everything.
    pyglet.app.run()