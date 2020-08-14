import React from "react";
import { getColor } from "../Sketch";
import styles from "../Sketch.module.scss";

export default function TopLayer(props) {
  const { status } = props;
  return (
    <svg
      className={styles.element}
      style={{
        height: "-webkit-fill-available",
      }}
      version="1.1"
      xmlns="http://www.w3.org/2000/svg"
      x="0"
      y="0"
      viewBox="0 0 437.1 754.7"
    >
      <g fill={getColor(status["jetson"])}>
        <path d="M102 467h90v77h-90z" />
        <path d="M191 466h1v2h-1zM102 466h82v3h-82z" />
      </g>
      <circle cx="218.4" cy="320.7" r="20" fill={getColor(status[1])} />
      <g fill={getColor(status[2])}>
        <path d="M208 348h21v35h-21zM209 383h20v30h-20zM208 414h22v5h-22z" />
        <path d="M209 413h20v20h-20zM211 434h15v9h-15zM210 433h18v1h-18zM209 433h1v1h-1z" />
        <path d="M209 433h1v1h-1zM228 433v1zM228 433h1v1h-1zM229 419v1zM228 419h1v1h-1zM228 413h1v1h-1zM229 414zM208 413h1v1h-1zM208 414h1v1h-1zM209 419v1zM208 419h1v1h-1z" />
      </g>
      <g fill={getColor(status[3])}>
        <path d="M158 271h42v21h-42zM128 273h30v18h-30zM107 272h21v20h-21zM98 274h9v16h-9z" />
      </g>
      <g fill={getColor(status[4])}>
        <path d="M207 107h19v25h-19z" />
        <path d="M207 105h1v4h-1zM208 105h5v3h-5zM213 106h1v4h-1zM222 105h2v4h-2z" />
        <path d="M223 105h1v3h-1zM224 107h1v1h-1zM225 107h1v24h-1zM222 131h2v2h-2zM207 131h2v2h-2zM205 110v20h2v-20l-1-1-1 1z" />
        <path d="M205 109l1-1h1v2h-1l-1-1z" />
      </g>
      <g fill={getColor(status[5])}>
        <path d="M211 98h14v5h-14zM214 103h8v1h-8zM211 103h14-14z" />
      </g>
      <g fill={getColor(status[6])}>
        <path d="M208 53h20v22h-20zM208 75h7-7zM208 75h1v1h-1z" />
        <path transform="rotate(-8 209 75)" d="M208 75h2v1h-2z" />
        <path d="M227 75h1v1h-1z" />
        <path
          transform="rotate(-87 224 75) scale(.99995)"
          d="M224 72h1v6h-1z"
        />
      </g>
      <path
        d="M374 311V158l1-1h44l1 1v153l-1 1h-44l-1-1z"
        fill={getColor(status["RF"])}
      />
      <path
        d="M17 311V158l1-1h44l1 1v153l-1 1H18l-1-1z"
        fill={getColor(status["LF"])}
      />
      <path
        d="M16 746V593l1-1h45l1 1v153l-1 1H18l-2-1z"
        fill={getColor(status["LB"])}
      />
      <path
        d="M374 746V593l1-1h44l1 1v153l-1 1h-44l-1-1z"
        fill={getColor(status["RB"])}
      />
    </svg>
  );
}
