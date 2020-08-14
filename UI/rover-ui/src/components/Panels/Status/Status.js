import React, { useState, useEffect } from "react";
import styles from "./Status.module.scss";
import Sketch from "../../RoverSketches/Sketch";
import { Box, Grid, Divider, ButtonGroup, Button } from "@material-ui/core";

import CheckCircleIcon from "@material-ui/icons/CheckCircle";

export default function Status() {
  const [state, setState] = useState({
    view: "all",
    RoverStatus: {
      jetson: "OK",
      LF: "OK",
      LB: "OK",
      RF: "OK",
      RB: "OK",
      1: "OK",
      2: "OK",
      3: "OK",
      4: "OK",
      5: "OK",
      6: "OK",
    },
  });

  useEffect(() => {
    const interval = setInterval(() => {
      let newStatus = {};
      // eslint-disable-next-line
      for (let [key, value] of Object.entries(state.RoverStatus)) {
        let r = Math.random();
        if (r < 0.05) newStatus[key] = "ERR";
        else if (r < 0.1) newStatus[key] = "WARN";
        else newStatus[key] = "OK";
      }
      setState({
        ...state,
        RoverStatus: newStatus,
      });
    }, 2000);
    return () => clearInterval(interval);
  }, [state]);

  const getStatusMessage = () => {
    let e = Object.values(state.RoverStatus).filter((x) => x === "ERR").length;
    let w = Object.values(state.RoverStatus).filter((x) => x === "WARN").length;
    if (e && w) {
      return `${e} error(s) and ${w} warning(s) found.`;
    } else if (e) {
      return `${e} error(s) found.`;
    } else if (w) {
      return `${w} warning(s) found.`;
    } else {
      return "No problems found.";
    }
  };

  const getColor = () => {
    let e = Object.values(state.RoverStatus).filter((x) => x === "ERR").length;
    let w = Object.values(state.RoverStatus).filter((x) => x === "WARN").length;
    return e ? styles.error : w ? styles.warn : styles.good;
  };

  return (
    <Grid container direction="row" justify="center" alignItems="stretch">
      <Grid
        item
        xs={6}
        style={{
          display: "flex",
          justifyContent: "center",
          alignItems: "stretch",
        }}
      >
        <Grid container direction="row" justify="center" alignItems="stretch">
          <Grid
            item
            xs={12}
            style={{
              display: "flex",
              justifyContent: "center",
              alignItems: "center",
              flexGrow: "1",
            }}
          >
            <ButtonGroup
              color="primary"
              aria-label="outlined primary button group"
            >
              <Button
                variant={state.view === "all" ? "contained" : ""}
                onClick={() => setState({ ...state, view: "all" })}
              >
                All
              </Button>
              <Button
                variant={state.view === "top" ? "contained" : ""}
                onClick={() => setState({ ...state, view: "top" })}
              >
                Top
              </Button>
              <Button
                variant={state.view === "arm" ? "contained" : ""}
                onClick={() => setState({ ...state, view: "arm" })}
              >
                Arm
              </Button>
              <Button
                variant={state.view === "science" ? "contained" : ""}
                onClick={() => setState({ ...state, view: "science" })}
              >
                Science
              </Button>
            </ButtonGroup>
          </Grid>
          <Grid
            item
            xs={12}
            style={{
              display: "flex",
              justifyContent: "center",
              alignItems: "center",
              padding: "0.8rem",
            }}
          >
            <Sketch view={state.view} status={state.RoverStatus} />
          </Grid>
        </Grid>
      </Grid>
      <Divider orientation="vertical" flexItem />
      <Grid
        item
        xs={6}
        style={{
          flex: 1,
        }}
      >
        <Box height="100%">
          <Grid
            container
            spacing={0}
            direction="column"
            alignItems="center"
            justify="center"
            className={getColor()}
          >
            <Grid item xs={12}>
              <CheckCircleIcon
                style={{
                  fontSize: "2em",
                }}
              />
            </Grid>
            <Grid item xs={12} style={{ textAlign: "center" }}>
              {getStatusMessage()}
            </Grid>
          </Grid>
        </Box>
      </Grid>
    </Grid>
  );
}
