#!/bin/sh
PACKAGE_NAME=$(basename $( cd $(dirname $0) ; cd .. ; pwd -P ) )
PACKAGE_PATH=$(rospack find $PACKAGE_NAME)
deployer-gnulinux -s "$PACKAGE_PATH/config/deploy.ops" -linfo
