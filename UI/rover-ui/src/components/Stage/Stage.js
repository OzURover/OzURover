import React, { useState } from "react";
import styles from "./Stage.module.scss";

import NavigationIcon from "@material-ui/icons/Navigation";
import MapIcon from "@material-ui/icons/Map";
import DriveEtaIcon from "@material-ui/icons/DriveEta";
import InfoIcon from "@material-ui/icons/Info";
import BuildIcon from "@material-ui/icons/Build";
import EcoIcon from "@material-ui/icons/Eco";
import FlightLandIcon from "@material-ui/icons/FlightLand";
import BatteryChargingFullIcon from "@material-ui/icons/BatteryChargingFull";
import TimelineIcon from "@material-ui/icons/Timeline";

import { Paper, AppBar, Tabs, Tab } from "@material-ui/core";

import TabPanel from "../TabPanel/TabPanel";
import {
  Navigation,
  Radar,
  Gate,
  Battery,
  Locomotion,
  RoboticArm,
  Science,
  Diagnostics,
  Status,
} from "../Panels";

import { makeStyles } from "@material-ui/core/styles";

const useTabStyles = makeStyles({
  root: {
    justifyContent: "center",
  },
  scroller: {
    flexGrow: "0",
  },
});

export default function Stage() {
  const [active, setActive] = useState(8);
  const classes = useTabStyles();

  const handleChange = (event, newValue) => {
    setActive(newValue);
  };

  const tabID = (index) => `scrollable-force-tabpanel-${index}`;

  return (
    <Paper className={styles.Stage} elevation={4}>
      <AppBar position="static" color="default">
        <Tabs
          value={active}
          onChange={handleChange}
          variant="scrollable"
          scrollButtons="auto"
          indicatorColor="primary"
          textColor="primary"
          classes={{ root: classes.root, scroller: classes.scroller }}
        >
          <Tab label="Navigation" icon={<NavigationIcon />} id={tabID(0)} />
          <Tab label="Radar" icon={<MapIcon />} id={tabID(1)} />
          <Tab label="Gate" icon={<FlightLandIcon />} id={tabID(2)} />
          <Tab
            label="Battery"
            icon={<BatteryChargingFullIcon />}
            id={tabID(3)}
          />
          <Tab label="Locomotion" icon={<DriveEtaIcon />} id={tabID(4)} />
          <Tab label="Robotic Arm" icon={<TimelineIcon />} id={tabID(5)} />
          <Tab label="Science" icon={<EcoIcon />} id={tabID(6)} />
          <Tab label="Diagnostics" icon={<BuildIcon />} id={tabID(7)} />
          <Tab label="Status" icon={<InfoIcon />} id={tabID(8)} />
        </Tabs>
      </AppBar>
      <TabPanel value={active} index={0} component={<Navigation />} />
      <TabPanel value={active} index={1} component={<Radar />} />
      <TabPanel value={active} index={2} component={<Gate />} />
      <TabPanel value={active} index={3} component={<Battery />} />
      <TabPanel value={active} index={4} component={<Locomotion />} />
      <TabPanel value={active} index={5} component={<RoboticArm />} />
      <TabPanel value={active} index={6} component={<Science />} />
      <TabPanel value={active} index={7} component={<Diagnostics />} />
      <TabPanel value={active} index={8} component={<Status />} />
    </Paper>
  );
}
