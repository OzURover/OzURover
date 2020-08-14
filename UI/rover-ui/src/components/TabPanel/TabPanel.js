import React from "react";
import { Box } from "@material-ui/core";
import styles from "./TabPanel.module.scss";

export default function TabPanel(props) {
  const { children, value, index, component, ...other } = props;
  return (
    <div
      className={styles.parent}
      style={{ display: value === index ? "flex" : "none" }}
      role="tabpanel"
      id={`scrollable-force-tabpanel-${index}`}
      {...other}
    >
      {value === index && (
        <Box className={styles.box} p={0}>
          {component}
        </Box>
      )}
    </div>
  );
}
