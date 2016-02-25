<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0"
	xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

	<xsl:output indent="yes" />
	<xsl:variable name="numVehicles" select="meta/numVehicles" />

	<xsl:template name="vehicle">
		<xsl:param name="i" />
		<xsl:if test="$i &lt; $numVehicles">
			<include>
				<uri>model://gcVehicle</uri>
				<name>vehicle<xsl:value-of select="$i" /></name>
			</include>
			<include>
				<uri>model://wheel</uri>
				<name>vehicle<xsl:value-of select="$i" />::wheel0</name>
			</include>
			<include>
				<uri>model://wheel</uri>
				<name>vehicle<xsl:value-of select="$i" />::wheel1</name>
			</include>
			<include>
				<uri>model://wheel</uri>
				<name>vehicle<xsl:value-of select="$i" />::wheel2</name>
			</include>
			<include>
				<uri>model://wheel</uri>
				<name>vehicle<xsl:value-of select="$i" />::wheel3</name>
			</include>
			<xsl:call-template name="vehicle">
				<xsl:with-param name="i" select="$i+1" />
			</xsl:call-template>
		</xsl:if>
	</xsl:template>

	<xsl:template match="/">
		<sdf version="1.5">
			<world name="GcWorld">
				<scene>
					<sky>
						<clouds>
							<speed>5.0</speed>
							<direction>1 0 0 </direction>
						</clouds>
					</sky>
				</scene>

				<include>
					<uri>model://sun</uri>
				</include>

				<model name="my_ground_plane">
					<pose>0 0 0 0 0 0</pose>
					<static>true</static>
					<link name="link">
						<pose>0 0 0 0 0 0</pose>
						<visual name="visual">
							<cast_shadows>true</cast_shadows>
							<geometry>
								<plane>
									<normal>0 0 1</normal>
									<size>500 500</size>
								</plane>
							</geometry>
							<material>
								<script>
									<uri>file://../data/gazono.material</uri>
									<name>Gazebo/Grass1</name>
								</script>
							</material>
						</visual>
					</link>
				</model>

				<xsl:call-template name="vehicle">
					<xsl:with-param name="i" select="0" />
				</xsl:call-template>

				<road name="circular_road">
					<width>7.5</width>
					<point>50.00 0.00 0.05</point>
					<point>49.61 6.27 0.05</point>
					<point>48.43 12.43 0.05</point>
					<point>46.49 18.41 0.05</point>
					<point>43.82 24.09 0.05</point>
					<point>40.45 29.39 0.05</point>
					<point>36.45 34.23 0.05</point>
					<point>31.87 38.53 0.05</point>
					<point>26.79 42.22 0.05</point>
					<point>21.29 45.24 0.05</point>
					<point>15.45 47.55 0.05</point>
					<point>9.37 49.11 0.05</point>
					<point>3.14 49.90 0.05</point>
					<point>-3.14 49.90 0.05</point>
					<point>-9.37 49.11 0.05</point>
					<point>-15.45 47.55 0.05</point>
					<point>-21.29 45.24 0.05</point>
					<point>-26.79 42.22 0.05</point>
					<point>-31.87 38.53 0.05</point>
					<point>-36.45 34.23 0.05</point>
					<point>-40.45 29.39 0.05</point>
					<point>-43.82 24.09 0.05</point>
					<point>-46.49 18.41 0.05</point>
					<point>-48.43 12.43 0.05</point>
					<point>-49.61 6.27 0.05</point>
					<point>-50.00 -0.00 0.05</point>
					<point>-49.61 -6.27 0.05</point>
					<point>-48.43 -12.43 0.05</point>
					<point>-46.49 -18.41 0.05</point>
					<point>-43.82 -24.09 0.05</point>
					<point>-40.45 -29.39 0.05</point>
					<point>-36.45 -34.23 0.05</point>
					<point>-31.87 -38.53 0.05</point>
					<point>-26.79 -42.22 0.05</point>
					<point>-21.29 -45.24 0.05</point>
					<point>-15.45 -47.55 0.05</point>
					<point>-9.37 -49.11 0.05</point>
					<point>-3.14 -49.90 0.05</point>
					<point>3.14 -49.90 0.05</point>
					<point>9.37 -49.11 0.05</point>
					<point>15.45 -47.55 0.05</point>
					<point>21.29 -45.24 0.05</point>
					<point>26.79 -42.22 0.05</point>
					<point>31.87 -38.53 0.05</point>
					<point>36.45 -34.23 0.05</point>
					<point>40.45 -29.39 0.05</point>
					<point>43.82 -24.09 0.05</point>
					<point>46.49 -18.41 0.05</point>
					<point>48.43 -12.43 0.05</point>
					<point>49.61 -6.27 0.05</point>
					<point>50.00 0.00 0.05</point>
					<material>
						<script>
							<uri>file://../data/gazono.material</uri>
							<name>Gazebo/Line</name>
						</script>
					</material>
				</road>

				<plugin name="chrono_gazebo" filename="libchrono_gazebo.so" />
			</world>
		</sdf>

	</xsl:template>

</xsl:stylesheet>