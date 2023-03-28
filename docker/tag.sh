#!/usr/bin/env bash

# find L4T_VERSION
source docker/l4t_version.sh

if [ $ARCH = "aarch64" ]; then
	# local container:tag name
	CONTAINER_VERSION="r$L4T_VERSION"

	# get remote container URL
	if [ $L4T_RELEASE -eq 32 ]; then
		if [[ $L4T_REVISION_MAJOR -lt 4 && $L4T_REVISION_MINOR -gt 4 ]]; then
			# L4T R32.4 was the first version containers are supported on
			echo "Docker containers aren't supported on this version of JetPack-L4T - please upgrade"
			exit 1
		elif [ $L4T_REVISION_MAJOR -eq 5 ]; then
			# L4T R32.5.x all run the R32.5.0 container
			CONTAINER_VERSION="r32.5.0"
		elif [ $L4T_REVISION_MAJOR -eq 7 ]; then
			# L4T R32.7.x all run the R32.7.0 container
			CONTAINER_VERSION="r32.7.1"
		fi
	elif [ $L4T_RELEASE -eq 35 ]; then
		if [ $L4T_REVISION_MAJOR -gt 2 ]; then
			CONTAINER_VERSION="r35.2.1"
		fi
	fi

elif [ $ARCH = "x86_64" ]; then
	echo "ros_deep_learning container is only supported/available on aarch64"
	exit 1
fi

