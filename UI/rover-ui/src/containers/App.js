import React from "react";
import styles from "./App.module.scss";
import Stage from "../components/Stage/Stage";

import Paper from "@material-ui/core/Paper";
import Container from "@material-ui/core/Container";
import AppBar from "@material-ui/core/AppBar";
import Toolbar from "@material-ui/core/Toolbar";
import Typography from "@material-ui/core/Typography";
import IconButton from "@material-ui/core/IconButton";
import PublicIcon from "@material-ui/icons/Public";

import useMediaQuery from "@material-ui/core/useMediaQuery";
import { ThemeProvider, createMuiTheme } from "@material-ui/core/styles";

function App() {
  const prefersDarkMode = useMediaQuery("(prefers-color-scheme: dark)");
  const theme = React.useMemo(
    () =>
      createMuiTheme({
        palette: {
          type: prefersDarkMode ? "dark" : "light",
          primary: {
            main: "#1D5B91",
          },
          secondary: {
            main: "#A93667",
          },
          divider: "#BDBDBD",
          tonalOffset: 0.2,
        },
      }),
    [prefersDarkMode]
  );

  const tileColor = prefersDarkMode
    ? theme.palette.primary.dark
    : theme.palette.primary.main;

  return (
    <ThemeProvider theme={theme}>
      <Paper
        className={styles.tile}
        style={{ backgroundColor: tileColor }}
        elevation={4}
        square
      >
        <AppBar
          position="sticky"
          style={{ backgroundColor: tileColor }}
          elevation={0}
        >
          <Toolbar>
            <IconButton edge="start" color="inherit">
              <PublicIcon />
            </IconButton>
            <Typography variant="h6" color="inherit" style={{ flexGrow: 1 }}>
              OzU Rover, <i><b>2020</b> UI Debut</i>
            </Typography>
          </Toolbar>
        </AppBar>
      </Paper>

      <Paper className={styles.App} elevation={0} square>
        <Container className={styles.container} maxWidth={false}>
          <Stage />
        </Container>
      </Paper>
    </ThemeProvider>
  );
}

export default App;
