import React from "react";
import styles from "./App.module.scss";
import Stage from "../components/Stage/Stage";

import {
  Paper,
  Container,
  AppBar,
  Toolbar,
  Typography,
  IconButton,
  useMediaQuery,
  ThemeProvider,
  createMuiTheme,
} from "@material-ui/core";
import PublicIcon from "@material-ui/icons/Public";

export default function App() {
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
              OzU Rover,{" "}
              <i>
                <b>2020</b> UI Debut
              </i>
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
