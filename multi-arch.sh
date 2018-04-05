#!/bin/sh

VERSION=$1

docker build -t openkorp/opendlv-device-kiwi-prugw-aarch64:$VERSION -f Dockerfile.aarch64 . &
docker build -t openkorp/opendlv-device-kiwi-prugw-amd64:$VERSION -f Dockerfile.amd64 . &
docker build -t openkorp/opendlv-device-kiwi-prugw-armhf:$VERSION -f Dockerfile.armhf .

docker push openkorp/opendlv-device-kiwi-prugw-aarch64:$VERSION 
docker push openkorp/opendlv-device-kiwi-prugw-amd64:$VERSION 
docker push openkorp/opendlv-device-kiwi-prugw-armhf:$VERSION 

cat <<EOF >/tmp/multi.yml
image: openkorp/opendlv-device-kiwi-prugw-multi:$VERSION
manifests:  
  - image: openkorp/opendlv-device-kiwi-prugw-amd64:$VERSION
    platform:
      architecture: amd64
      os: linux
  - image: openkorp/opendlv-device-kiwi-prugw-armhf:$VERSION
    platform:
      architecture: arm
      os: linux
  - image: openkorp/opendlv-device-kiwi-prugw-aarch64:$VERSION
    platform:
      architecture: arm64
      os: linux
EOF
manifest-tool-linux-amd64 push from-spec /tmp/multi.yml
