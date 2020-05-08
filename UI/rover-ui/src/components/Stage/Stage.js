import React, { useState } from "react";
import styles from "./Stage.module.scss";

import Paper from "@material-ui/core/Paper";
import AppBar from "@material-ui/core/AppBar";
import Tabs from "@material-ui/core/Tabs";
import Tab from "@material-ui/core/Tab";

import TabPanel from "../TabPanel/TabPanel";

import NavigationIcon from "@material-ui/icons/Navigation";
import DriveEtaIcon from '@material-ui/icons/DriveEta';
import InfoIcon from '@material-ui/icons/Info';

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
  const [active, setActive] = useState(0);
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
          <Tab label="Locomotion" icon={<DriveEtaIcon />} id={tabID(1)} />
          <Tab label="Status" icon={<InfoIcon />} id={tabID(2)} />
        </Tabs>
      </AppBar>
      <TabPanel value={active} index={0}>
        Item One
      </TabPanel>
      <TabPanel value={active} index={1}>
        Item Two
      </TabPanel>
      <TabPanel value={active} index={2}>
        Item Three
      </TabPanel>
    </Paper>
  );
}
