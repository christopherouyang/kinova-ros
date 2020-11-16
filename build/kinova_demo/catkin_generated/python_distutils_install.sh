#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/wmrm/kinova-ros/src/kinova_demo"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/wmrm/kinova-ros/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/wmrm/kinova-ros/install/lib/python2.7/dist-packages:/home/wmrm/kinova-ros/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/wmrm/kinova-ros/build" \
    "/usr/bin/python2" \
    "/home/wmrm/kinova-ros/src/kinova_demo/setup.py" \
     \
    build --build-base "/home/wmrm/kinova-ros/build/kinova_demo" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/wmrm/kinova-ros/install" --install-scripts="/home/wmrm/kinova-ros/install/bin"
