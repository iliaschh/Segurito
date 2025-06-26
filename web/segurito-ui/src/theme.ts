import { createTheme } from "@mui/material";

export const theme = createTheme({
  palette: {
    mode: "dark",
    background: {
      default: "#121212",     // fondo global
      paper:    "#1e1e1e",    // tarjetas
    },
    primary:   { main: "#00b894" },
    secondary: { main: "#0984e3" },
  },
  typography: {
    button: { textTransform: "none", fontWeight: 600 },
  },
});

