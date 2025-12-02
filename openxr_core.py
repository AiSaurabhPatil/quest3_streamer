import xr

def create_openxr_instance():
    print("Initializing OpenXR...")
    requested_extensions = [
        xr.KHR_OPENGL_ENABLE_EXTENSION_NAME,
        xr.MNDX_EGL_ENABLE_EXTENSION_NAME,
    ]
    
    return xr.create_instance(
        xr.InstanceCreateInfo(
            application_info=xr.ApplicationInfo(
                application_name="Quest3DataStreamerGL",
                application_version=1,
                engine_name="PyOpenXR",
                engine_version=1,
                api_version=xr.XR_CURRENT_API_VERSION,
            ),
            enabled_extension_names=requested_extensions,
        )
    )

def get_openxr_system(instance):
    return xr.get_system(
        instance,
        xr.SystemGetInfo(
            form_factor=xr.FormFactor.HEAD_MOUNTED_DISPLAY,
        )
    )
