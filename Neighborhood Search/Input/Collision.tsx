<?xml version="1.0" encoding="UTF-8"?>
<tileset version="1.2" tiledversion="1.3.2" name="Collision" tilewidth="32" tileheight="32" tilecount="18" columns="18">
 <image source="../../../Textures/collisionmask 32.png" width="576" height="32"/>
 <tile id="2">
   <properties>
    <property name="isSlope" type="bool" value="true"/>
    <property name="slopeTopLeft" type="int" value="15"/>
    <property name="slopeTopRight" type="int" value="0"/>
   </properties>
   <objectgroup draworder="index"/>
  </tile>
  <tile id="3">
   <properties>
    <property name="isSlope" type="bool" value="true"/>
    <property name="slopeTopLeft" type="int" value="0"/>
    <property name="slopeTopRight" type="int" value="15"/>
   </properties>
   <objectgroup draworder="index"/>
  </tile>
  <tile id="4">
   <properties>
    <property name="isSlope" type="bool" value="true"/>
    <property name="slopeTopLeft" type="int" value="15"/>
    <property name="slopeTopRight" type="int" value="8"/>
   </properties>
   <objectgroup draworder="index"/>
  </tile>
  <tile id="5">
   <properties>
    <property name="isSlope" type="bool" value="true"/>
    <property name="slopeTopLeft" type="int" value="7"/>
    <property name="slopeTopRight" type="int" value="0"/>
   </properties>
   <objectgroup draworder="index"/>
  </tile>
  <tile id="6">
   <properties>
    <property name="isSlope" type="bool" value="true"/>
    <property name="slopeTopLeft" type="int" value="0"/>
    <property name="slopeTopRight" type="int" value="7"/>
   </properties>
   <objectgroup draworder="index"/>
  </tile>
  <tile id="7">
   <properties>
    <property name="isSlope" type="bool" value="true"/>
    <property name="slopeTopLeft" type="int" value="8"/>
    <property name="slopeTopRight" type="int" value="15"/>
   </properties>
   <objectgroup draworder="index"/>
  </tile>
  <tile id="15">
   <properties>
    <property name="isOneWayPlatform" type="bool" value="true"/>
   </properties>
   <objectgroup draworder="index"/>
  </tile>
</tileset>
