import React from "react";
import { Container } from "@material-ui/core";
import styles from "./Sketch.module.scss";

import TopLayer from "./components/TopLayer";
import { ReactComponent as TopBase } from "./sketches/top.svg";

export const getColor = (name) => {
  switch (name) {
    case "ERR":
      return "#fc1c03";
    case "WARN":
      return "#ffd000";
    case "OK":
      return "#2be007";
    default:
      return "#000000";
  }
};

export default function Sketch(props) {
  const { view, status } = props;

  return (
    <Container className={styles.parent}>
      {view === "all" && (
        <div className={styles.container}>
          <TopLayer status={status} />
          <TopBase className={styles.element} />
        </div>
      )}
      {view === "top" && (
        <div className={styles.container}>
          <TopLayer status={status} />
          <TopBase className={styles.element} />
        </div>
      )}
      {view === "arm" && (
        <div className={styles.container}>
          <TopLayer status={status} />
          <TopBase className={styles.element} />
        </div>
      )}
      {view === "science" && (
        <div className={styles.container}>
          <TopLayer status={status} />
          <TopBase className={styles.element} />
        </div>
      )}
    </Container>
  );
}