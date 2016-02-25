# Generate world file
meta = ../../data/metadata.xml
world = trafficCircle

all: trafficCircle.world

$(world).world: $(meta) $(world).xsl
	xsltproc $(world).xsl $(meta) > $(world).world