<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0"
	xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

	<xsl:output indent="yes" />
	<xsl:variable name="numVehicles" select="metadata/numVehicles" />

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

				<light type="directional" name="sun">
					<pose>100 0 100 0 -0.785 0</pose>
					<diffuse>1 1 1 1</diffuse>
					<specular>.1 .1 .1 1</specular>
					<attenuation>
						<range>5000</range>
						<linear>0.2</linear>
						<constant>0.8</constant>
						<quadratic>0.01</quadratic>
					</attenuation>
					<cast_shadows>true</cast_shadows>
				</light>

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
									<name>Gazebo/Ground01</name>
								</script>

							</material>
						</visual>
						<!--
						<sensor name="topcamera" type="camera">
							<pose>-70 0 30 0 .62 0</pose>
							<camera name="secCam">
								<horizontal_fov>1.57</horizontal_fov>
								<image>
									<format>B8G8R8</format>
									<width>2560</width>
									<height>1440</height>
								</image>
								<clip>
									<near>0.1</near>
									<far>500</far>
								</clip>
								<noise>
									<type>gaussian</type>
									<mean>0.0</mean>
									<stddev>0.000</stddev>
								</noise>
								<save enabled="true">
									<path>Captures/SecCam/2016_03_02_2k</path>
								</save>
							</camera>
							<always_on>1</always_on>
							<update_rate>60</update_rate>
							<visualize>true</visualize>
						</sensor>
					-->
					<!--
						<sensor name="sidecamera" type="camera">
							<pose>-55 0 2 0 .2 .6</pose>
							<camera name="sideCam">
								<horizontal_fov>1.57</horizontal_fov>
								<image>
									<format>B8G8R8</format>
									<width>2560</width>
									<height>1440</height>
								</image>
								<clip>
									<near>0.1</near>
									<far>500</far>
								</clip>
								<noise>
									<type>gaussian</type>
									<mean>0.0</mean>
									<stddev>0.000</stddev>
								</noise>
								<save enabled="true">
									<path>Captures/SideCam/2016_03_02_2k</path>
								</save>
							</camera>
							<always_on>1</always_on>
							<update_rate>60</update_rate>
							<visualize>true</visualize>
						</sensor>
					-->
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
