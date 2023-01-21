/** @type {import('tailwindcss').Config} */
module.exports = {
  content: ["./src/**/*.{js,jsx,ts,tsx}"],
  theme: {
    extend: {
      fontFamily: {
        // 'sans': ['ui-sans-serif', 'system-ui', ...],
        // 'serif': ['ui-serif', 'Georgia', ...],
        mono: ["ui-monospace", "SFMono-Regular"],
        // 'display': ['Oswald', ...],
        // 'body': ['"Open Sans"', ...],
      },
      height: {
        "1/10": "10%",
        "7/10": "70%",
        "8/10": "80%",
        "9/10": "90%",
      },
      width: {
        "9/10": "90%",
      },
    },
  },
  plugins: [],
};
