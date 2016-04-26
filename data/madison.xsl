<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
  <xsl:output method="xml" indent="yes" omit-xml-declaration="no"/>
  <xsl:strip-space elements="*"/>

  <xsl:template match="@* | node()">
    <xsl:copy>
      <xsl:apply-templates select="@* | node()"/>
    </xsl:copy>
  </xsl:template>

  <xsl:template match="osm">
    <xsl:copy>
      <xsl:apply-templates select="bounds" />
      <xsl:apply-templates select="node | way | relation">
        <xsl:sort select="@id" data-type="number" order="ascending"/>
      </xsl:apply-templates>
    </xsl:copy>
  </xsl:template>
</xsl:stylesheet>
