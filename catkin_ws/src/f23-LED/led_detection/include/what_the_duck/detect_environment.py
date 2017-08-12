
def on_duckiebot():
    import platform
    proc = platform.processor()
    on_the_robot = not('x86' in proc)
    # armv7l
    return on_the_robot