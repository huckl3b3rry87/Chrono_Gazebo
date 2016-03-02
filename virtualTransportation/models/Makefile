# Generate world file
meta = ../../data/metadata.xml
world = trafficCircle

all: $(world).world

$(world).world: $(meta) $(world).xsl
	xsltproc $(world).xsl $(meta) > $(world).world