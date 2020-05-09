import React from "react";
import Box from "@material-ui/core/Box";

export default function TabPanel(props) {
  const { children, value, index, component, ...other } = props;
  return (
    <div
      role="tabpanel"
      hidden={value !== index}
      id={`scrollable-force-tabpanel-${index}`}
      {...other}
    >
      {value === index && (
        <Box p={3}>
          {component}
        </Box>
      )}
    </div>
  );
}
