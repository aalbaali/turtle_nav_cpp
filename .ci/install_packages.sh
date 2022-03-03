apt-get -y update
xargs apt-get -y install --no-install-recommends < .ci/packages.txt
