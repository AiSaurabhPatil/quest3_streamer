import glfw
import ctypes
import sys
import xr
import xr.platform.linux
import OpenGL.raw.GLX._types as GLXTypes

# Attempt to load libGL and libglfw
try:
    libGL = ctypes.CDLL("libGL.so.1")
    libGL.glXGetCurrentContext.restype = ctypes.c_void_p
    libGL.glXGetCurrentDisplay.restype = ctypes.c_void_p
    libGL.glXGetCurrentDrawable.restype = ctypes.c_ulong
except OSError:
    print("Could not load libGL.so.1. Are you on a system without GLX?")
    sys.exit(1)

libglfw_names = ['libglfw.so', 'libglfw.so.3', 'libglfw3.so']
libglfw = None
for name in libglfw_names:
    try:
        libglfw = ctypes.CDLL(name)
        break
    except OSError:
        pass

if not libglfw:
    print("Warning: Could not load libglfw.so. Native handles might fail.")

def init_glfw():
    print("Initializing GLFW...")
    if not glfw.init():
        raise RuntimeError("Failed to init GLFW")

    glfw.window_hint(glfw.VISIBLE, True)
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 2)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    
    window = glfw.create_window(640, 480, "Quest Stream", None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("Failed to create window")

    glfw.make_context_current(window)
    return window

def create_graphics_binding(libglfw, libGL):
    wl_display = None
    if hasattr(libglfw, 'glfwGetWaylandDisplay'):
        libglfw.glfwGetWaylandDisplay.restype = ctypes.c_void_p
        wl_display = libglfw.glfwGetWaylandDisplay()
    
    x_display = None
    if hasattr(libGL, 'glXGetCurrentDisplay'):
        x_display = libGL.glXGetCurrentDisplay()

    if wl_display:
        print("Using Wayland Binding")
        wl_display_ptr = ctypes.cast(wl_display, ctypes.POINTER(xr.platform.linux.wl_display))
        return xr.GraphicsBindingOpenGLWaylandKHR(display=wl_display_ptr)
    elif x_display:
        print("Using X11 Binding")
        glx_context = libGL.glXGetCurrentContext()
        glx_drawable = libGL.glXGetCurrentDrawable()
        
        x_display_ptr = ctypes.cast(x_display, ctypes.POINTER(GLXTypes.struct__XDisplay)) if x_display else None
        glx_context_ptr = ctypes.cast(glx_context, ctypes.POINTER(GLXTypes.struct___GLXcontextRec)) if glx_context else None
        
        return xr.GraphicsBindingOpenGLXlibKHR(
            x_display_ptr,
            0,
            ctypes.POINTER(GLXTypes.struct___GLXFBConfigRec)(),
            glx_drawable,
            glx_context_ptr,
        )
    else:
        raise RuntimeError("No valid graphics handles found")
