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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/bigrig/Sentinel/src/rosserial/rosserial_xbee"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/bigrig/Sentinel/install/lib/python2.7/dist-packages:/home/bigrig/Sentinel/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/bigrig/Sentinel/build" \
    "/usr/bin/python" \
    "/home/bigrig/Sentinel/src/rosserial/rosserial_xbee/setup.py" \
    build --build-base "/home/bigrig/Sentinel/build/rosserial/rosserial_xbee" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/bigrig/Sentinel/install" --install-scripts="/home/bigrig/Sentinel/install/bin"
