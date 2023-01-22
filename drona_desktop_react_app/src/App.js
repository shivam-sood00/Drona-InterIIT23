import React, { useState, useEffect } from "react";
import axios from "axios";
import logo from "./logo.svg";
import "./App.css";
import takeOffButton from "./Assets/Images/takeOff.png";
import landButton from "./Assets/Images/land.png";
import armButton from "./Assets/Images/arm.png";
import trapezium from "./Assets/Images/trapezium.png";
import aButton from "./Assets/Images/a.png";
import dButton from "./Assets/Images/d.png";
import sButton from "./Assets/Images/s.png";
import wButton1 from "./Assets/Images/w_new.png";
import leftButton from "./Assets/Images/left.png";
import downButton from "./Assets/Images/down.png";
import rightButton from "./Assets/Images/right.png";
import upButton from "./Assets/Images/up.png";
import wasd from "./Assets/Images/wasd.png";
import UpRightDownLeft from "./Assets/Images/UpRightDownLeft.png";
import graphNameBg from "./Assets/Images/graph_name_bg.png";
import graphLegendBg from "./Assets/Images/graph_legend_bg.png";
import graph_group_name_active_bg from "./Assets/Images/graph_group_name_active_bg.png";

import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
} from "chart.js";
import { Line } from "react-chartjs-2";
import faker from "faker";

ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
);

function App() {
  const url = `http://127.0.0.1:5001/`; // URL for FLASK Server

  // STATES
  const [response, setResponse] = useState(null);
  const [time, setTime] = useState(new Date());
  const [throttle, setThrottle] = useState(-1); // Instantaneous Throttle of the drone
  const [roll, setRoll] = useState(-1); // Instantaneous Roll of the drone
  const [pitch, setPitch] = useState(-1); // Instantaneous Pitch of the drone
  const [yaw, setYaw] = useState(-1); // Instantaneous Yaw of the drone
  const [graphToggle, setGraphToggle] = useState(true);
  const [landTakeOff, setLandTakeOff] = useState(true);
  // data1 => (xData1, yData1, zData1) = (YAW, ROLL, PITCH)

  const [xData1, setXData1] = useState([]);
  const [yData1, setYData1] = useState([]);
  const [zData1, setZData1] = useState([]);

  // data2 => (xData2) = (THROTTLE)
  const [xData2, setXData2] = useState([]);

  // labelData => seconds passed
  const [labelData1, setLabelData1] = useState([0, 1, 2, 3, 4, 5, 6]);
  const [labelData2, setLabelData2] = useState([0, 1, 2, 3, 4, 5, 6]);

  // API to control drone
  const callYourAPI = (e) => {
    let requestURL = url + "controller/" + e;

    if (e == "disArm") setThrottle((prev) => 0); // Setting throttle to 0 at disArm
    if (e == "takeOff") setLandTakeOff((prev) => !prev);
    axios.get(requestURL).then((res) => {
      setResponse(res);
    });
  };

  // API to get drone parameters i.e Throttle, Roll, Pitch, Yaw
  const droneParameterAPI = () => {
    let requestURL = url + "drone_param";

    axios.get(requestURL).then((res) => {
      let resData = res.data;
      setThrottle((prev) => resData["rcThrottle"]);
      setRoll((prev) => resData["Roll"]);
      setPitch((prev) => resData["Pitch"]);
      setYaw((prev) => resData["Yaw"]);
    });
  };

  const options = {
    plugins: {
      legend: {
        display: false,
      },
    },
    scales: {
      x: {
        ticks: {
          display: false,
        },
        grid: {
          display: false,
        },
      },
      y: {
        grid: {
          display: true,
          color: "#949494",
        },
      },
    },
  };

  const [data1, setData1] = useState({
    labels: labelData1,
    datasets: [
      {
        label: "YAW",
        data: xData1,
        fill: false,
        backgroundColor: "rgba(75,192,192,0.2)",
        borderColor: "#ffde59",
        lineTension: 0.4,
        radius: 6,
      },
      {
        label: "ROLL",
        data: yData1,
        fill: false,
        backgroundColor: "rgba(75,192,192,0.2)",
        borderColor: "#5ce1e6",
        lineTension: 0.4,
        radius: 6,
      },
      {
        label: "PITCH",
        data: zData1,
        fill: false,
        backgroundColor: "rgba(75,192,192,0.2)",
        borderColor: "#ffa7e3",
        lineTension: 0.4,
        radius: 6,
      },
    ],
  });

  const [data2, setData2] = useState({
    labels: labelData1,
    datasets: [
      {
        label: "Throttle",
        data: xData2,
        fill: false,
        backgroundColor: "rgba(75,192,192,0.2)",
        borderColor: "#ffde59",
        lineTension: 0.4,
        radius: 6,
      },
    ],
  });

  // Updating both Charts
  const updateChart = () => {
    removeData(xData1, setXData1, yaw);
    removeData(yData1, setYData1, roll);
    removeData(zData1, setZData1, pitch);

    removeData(xData2, setXData2, throttle);

    // Updating seconds
    removeLabel(labelData1, setLabelData1);
    removeLabel(labelData2, setLabelData2);

    setData1((prev) => ({
      labels: labelData1,
      datasets: [
        {
          label: prev.datasets[0].label,
          data: xData1,
          fill: prev.datasets[0].fill,
          backgroundColor: prev.datasets[0].backgroundColor,
          borderColor: prev.datasets[0].borderColor,
        },
        {
          label: prev.datasets[1].label,
          data: yData1,
          fill: prev.datasets[1].fill,
          backgroundColor: prev.datasets[1].backgroundColor,
          borderColor: prev.datasets[1].borderColor,
        },
        {
          label: prev.datasets[2].label,
          data: zData1,
          fill: prev.datasets[2].fill,
          backgroundColor: prev.datasets[2].backgroundColor,
          borderColor: prev.datasets[2].borderColor,
        },
      ],
    }));

    setData2((prev) => ({
      labels: labelData2,
      datasets: [
        {
          label: prev.datasets[0].label,
          data: xData2,
          fill: prev.datasets[0].fill,
          backgroundColor: prev.datasets[0].backgroundColor,
          borderColor: prev.datasets[0].borderColor,
        },
      ],
    }));
  };

  // Adding instantaneous data to the chart
  const addData = (axisData, setAxisData, val) => {
    setAxisData((prev) => [...prev, val]);
  };

  // Removing old data from the chart
  const removeData = (axisData, setAxisData, val) => {
    if (axisData.length < 10) addData(axisData, setAxisData, val);
    else
      setAxisData(
        (prev) => prev.filter((a, idx) => (idx !== 0 ? a : "")),
        addData(axisData, setAxisData, val)
      );
  };

  const addLabel = (labelData, setLabelData) => {
    setLabelData((prev) => [...prev, labelData[labelData.length - 1] + 1]);
  };

  const removeLabel = (labelData, setLabelData) => {
    if (labelData.length < 9) addLabel(labelData, setLabelData);
    else
      setLabelData(
        (prev) => prev.filter((a, idx) => (idx !== 0 ? a : "")),
        addLabel(labelData, setLabelData)
      );
  };

  useEffect(() => {
    const interval = setInterval(() => {
      droneParameterAPI();
      removeLabel(labelData1, setLabelData1);
      removeLabel(labelData2, setLabelData2);
      updateChart();
      setTime(new Date());
    }, 20);

    return () => clearInterval(interval);
  }, [xData1, yData1, zData1, xData2, labelData1, labelData2]);

  // Event Listeners to controll using Keyboard
  useEffect(() => {
    document.body.addEventListener("keydown", function (event) {
      if (event.keyCode == 87) callYourAPI("increaseHeight");
      else if (event.keyCode == 65) callYourAPI("leftYaw");
      else if (event.keyCode == 83) callYourAPI("decreaseHeight");
      else if (event.keyCode == 68) callYourAPI("rightYaw");
      else if (event.keyCode == 38) callYourAPI("forward");
      else if (event.keyCode == 37) callYourAPI("left");
      else if (event.keyCode == 40) callYourAPI("backward");
      else if (event.keyCode == 39) callYourAPI("right");
      else if (event.keyCode == 81) callYourAPI("takeOff");
      else if (event.keyCode == 188) callYourAPI("arm");
      else if (event.keyCode == 190) callYourAPI("disArm");
      else if (event.keyCode == 69) callYourAPI("land");
    });
  }, []);

  return (
    <div className="App text-white flex">
      <div className="h-screen w-[50vw] mx-auto bg-[url('./Assets/Images/background.png')] bg-top bg-cover">
        <div className="grid grid-rows-1 grid-flow-col h-1/10 bg-[url('./Assets/Images/black_rectangle.png')] border-b-2 border-white">
          <div className="flex justify-evenly items-center">
            <div className="w-[20%] font-bold text-2xl"></div>
            <div className="w-[55%] flex justify-center relative top-[10%] w-[55vw] max-w-lg">
              <div>
                <img src={trapezium} alt="trapezium background" />
              </div>
              <div className="absolute flex items-center h-full font-bold text-3xl">
                CONNECTED
              </div>
            </div>
            <div className="w-[20%] font-bold text-2xl"></div>
          </div>
        </div>
        <div className="grid grid-rows-1 grid-flow-col h-7/10">
          <div className="flex justify-evenly">
            <div className="flex justify-center flex-col max-w-[40%]">
              <div className="relative">
                <img src={wasd} alt="wasd" />
                <button
                  value="increaseHeight"
                  onClick={(e) => callYourAPI(e.target.value)}
                  className="w-[23.4375%] h-[23.4375%] absolute left-[38.2%] top-[18.4%] rounded-full border border-white bg-red-500 opacity-0"
                >
                  w
                </button>
                <button
                  value="leftYaw"
                  onClick={(e) => callYourAPI(e.target.value)}
                  className="w-[23.4375%] h-[23.4375%] absolute left-[58.4%] top-[38.3%] rounded-full border border-white bg-red-500 opacity-0"
                >
                  a
                </button>
                <button
                  value="decreaseHeight"
                  onClick={(e) => callYourAPI(e.target.value)}
                  className="w-[23.4375%] h-[23.4375%] absolute left-[38.2%] top-[58.2%] rounded-full border border-white bg-red-500 opacity-0"
                >
                  s
                </button>
                <button
                  value="rightYaw"
                  onClick={(e) => callYourAPI(e.target.value)}
                  className="w-[23.4375%] h-[23.4375%] absolute left-[18.2%] top-[38.3%] rounded-full border border-white bg-red-500 opacity-0"
                >
                  d
                </button>
              </div>
            </div>
            <div className="flex justify-around flex-col max-w-[20%] w-[20%]">
              <div className="grid grid-rows-1 grid-flow-col border-2 border-dashed rounded-lg">
                <div className="flex flex-col ">
                  <div className="drop-shadow-xl h-10 w-9/10 my-2 mx-auto rounded bg-[#545454] border border-white">
                    {throttle}
                  </div>
                  <div className="text-2xl">THROTTLE</div>
                </div>
              </div>
              <div className="grid grid-rows-1 grid-flow-col border-2 border-dashed rounded-lg">
                <div className="flex flex-col ">
                  <div className="drop-shadow-xl h-10 w-9/10 my-2 mx-auto rounded bg-[#545454] border border-white">
                    {roll}
                  </div>
                  <div className="text-2xl">ROLL</div>
                </div>
              </div>
              <div className="grid grid-rows-1 grid-flow-col border-2 border-dashed rounded-lg">
                <div className="flex flex-col ">
                  <div className="drop-shadow-xl h-10 w-9/10 my-2 mx-auto rounded bg-[#545454] border border-white">
                    {pitch}
                  </div>
                  <div className="text-2xl">PITCH</div>
                </div>
              </div>
              <div className="grid grid-rows-1 grid-flow-col border-2 border-dashed rounded-lg">
                <div className="flex flex-col ">
                  <div className="drop-shadow-xl h-10 w-9/10 my-2 mx-auto rounded bg-[#545454] border border-white">
                    {yaw}
                  </div>
                  <div className="text-2xl">YAW</div>
                </div>
              </div>
            </div>
            <div className="flex justify-center flex-col max-w-[40%]">
              <div className="relative">
                <img src={UpRightDownLeft} alt="Up Right Down Left Button" />
                <button
                  value="forward"
                  onClick={(e) => callYourAPI(e.target.value)}
                  className="w-[23.4375%] h-[23.4375%] absolute left-[38.2%] top-[18.4%] rounded-full border border-white bg-red-500 opacity-0"
                >
                  ⬆️
                </button>
                <button
                  value="right"
                  onClick={(e) => callYourAPI(e.target.value)}
                  className="w-[23.4375%] h-[23.4375%] absolute left-[58.4%] top-[38.3%] rounded-full border border-white bg-red-500 opacity-0"
                >
                  ➡️
                </button>
                <button
                  value="backward"
                  onClick={(e) => callYourAPI(e.target.value)}
                  className="w-[23.4375%] h-[23.4375%] absolute left-[38.2%] top-[58.2%] rounded-full border border-white bg-red-500 opacity-0"
                >
                  ⬇️
                </button>
                <button
                  value="left"
                  onClick={(e) => callYourAPI(e.target.value)}
                  className="w-[23.4375%] h-[23.4375%] absolute left-[18.2%] top-[38.3%] rounded-full border border-white bg-red-500 opacity-0"
                >
                  ⬅️
                </button>
              </div>
            </div>
          </div>
        </div>
        <div className="grid grid-flow-col grid-col-2 h-1/5">
          <div className="flex h-8/10">
            <div className="w-4/12 flex justify-around items-center relative">
              <div className="h-4/5 relative">
                <button
                  value="takeOff"
                  onClick={(e) => callYourAPI(e.target.value)}
                  className="absolute w-full h-full left-0 "
                ></button>
                <img
                  className="mx-auto h-full"
                  src={landTakeOff ? landButton : takeOffButton}
                  alt="takeOffButton"
                />
              </div>
            </div>

            <div className="w-4/12"></div>
            <div className="w-4/12 flex justify-around items-center relative">
              <div className="h-4/5 relative">
                <button
                  value="arm"
                  onClick={(e) => callYourAPI(e.target.value)}
                  className="absolute w-full h-full left-0 "
                ></button>
                <img
                  className="mx-auto h-full"
                  src={armButton}
                  alt="armButton"
                />
              </div>
            </div>
          </div>
        </div>
      </div>
      <div className="h-screen w-[50vw] mx-auto bg-[url('./Assets/Images/graph_bg.png')] bg-top bg-cover">
        <div className="flex flex-col justify-around h-[95vh] items-center">
          {graphToggle ? (
            <div className="grid grid-rows-1 grid-flow-col h-[44vh] w-[90%] ">
              <div className="flex flex-col justify-center items-center w-[99%]">
                <div className="w-[100%] h-[10%] pl-4 font-bold text-2xl h-full flex  items-center bg-[url('./Assets/Images/graph_name_bg.png')] bg-no-repeat">
                  YAW
                </div>
                <div className="w-[100%] h-[10%] lg:h-[13%] pl-4 font-bold text-2xl flex  items-center bg-[url('./Assets/Images/graph_legend_bg.png')] bg-no-repeat bg-contain">
                  <div className="border-2 border-white mr-4 rounded h-[50%] lg:h-[70%] items-center flex text-xs lg:text-xl px-1 lg:px-1.5 font-thin">
                    Unit: degree(°)
                  </div>
                  <div className="border-2 border-[#ffde59] mr-4 rounded h-[50%] lg:h-[70%] items-center flex text-xs lg:text-xl px-1 lg:px-1.5 font-thin">
                    YAW: {yaw}
                  </div>
                  <div className="border-2 border-[#5ce1e6] mr-4 rounded h-[50%] lg:h-[70%] items-center flex text-xs lg:text-xl px-1 lg:px-1.5 font-thin">
                    ROLL: {roll}
                  </div>
                  <div className="border-2 border-[#ffa7e3] mr-4 rounded h-[50%] lg:h-[70%] items-center flex text-xs lg:text-xl px-1 lg:px-1.5 font-thin">
                    PITCH: {pitch}
                  </div>
                </div>
                <div className="w-[100%] h-[77%] font-bold text-2xl h-full flex items-center justify-center bg-[#545454] border-2 border-white rounded-tr-xl rounded-b-xl">
                  <Line data={data1} options={options} />
                </div>
              </div>
            </div>
          ) : null}
          {graphToggle ? (
            <div className="grid grid-rows-1 grid-flow-col h-[44vh] w-[90%]">
              <div className="flex flex-col justify-evenly items-center w-[99%]">
                <div className="w-[100%] h-[10%] pl-4 font-bold text-2xl h-full flex  items-center bg-[url('./Assets/Images/graph_name_bg.png')] bg-no-repeat">
                  THROTTLE
                </div>
                <div className="w-[100%] h-[13%] pl-4 font-bold text-2xl h-full flex  items-center bg-[url('./Assets/Images/graph_legend_bg.png')] bg-no-repeat bg-contain">
                  <div className="border-2 border-[#ffde59] mr-4 rounded h-[70%] items-center flex text-xl px-1.5 font-thin">
                    THROTTLE: {throttle}
                  </div>
                </div>
                <div className="w-[100%] h-[77%] font-bold text-2xl h-full flex items-center justify-center bg-[#545454] border-2 border-white rounded-tr-xl rounded-b-xl">
                  <Line data={data2} options={options} />
                </div>
              </div>
            </div>
          ) : null}
        </div>
      </div>
    </div>
  );
}

export default App;
