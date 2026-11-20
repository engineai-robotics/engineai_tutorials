"""Use ``from pm01_navigation_client import Navigator`` inside a ROS2 environment."""
def __getattr__(name):
    if name == 'Navigator':
        from .navigator import Navigator
        return Navigator
    raise AttributeError(name)
