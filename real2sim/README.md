# Real2Sim: Depth-Regulaized PGSR Reconstruction

Ref: 

[PGSR: Planar-based Gaussian Splatting Reconstruction](https://github.com/zju3dv/PGSR)

[3D Gaussian Splatting for Real-Time Radiance Field Rendering](https://github.com/graphdeco-inria/gaussian-splatting/tree/dev?tab=readme-ov-file#depth-regularization)

[Depth Pro: Sharp Monocular Metric Depth in Less Than a Second](https://github.com/apple/ml-depth-pro)  

## Depth-Regulaized PGSR Reconstruction

```latex

\begin{figure*}[htb]
    \centering
    \addtolength{\tabcolsep}{-6.5pt}
    \footnotesize{
        \setlength{\tabcolsep}{1pt} % Default value: 6pt
        \begin{tabular}{p{8.2pt}ccccc}
            & \textbf{GauStudio} & \textbf{SuGaR} & \textbf{PGSR} & \textbf{Ours} & \textbf{GT} \\

            \raisebox{23pt}{\rotatebox[origin=c]{90}{Screen}}&
            \includegraphics[width=0.18\textwidth]{./img/render_result/gs/0112.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/sugar/0112.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/pgsr/0112.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/pgsr_dn/0112.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/gt/0112.png} \\

            \raisebox{23pt}{\rotatebox[origin=c]{90}{Door}}&
            \includegraphics[width=0.18\textwidth]{./img/render_result/gs/0016.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/sugar/0016.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/pgsr/0016.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/pgsr_dn/0016.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/gt/0016.png} \\

            \raisebox{23pt}{\rotatebox[origin=c]{90}{Wall}}&
            \includegraphics[width=0.18\textwidth]{./img/render_result/gs/0019.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/sugar/0019.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/pgsr/0019.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/pgsr_dn/0019.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/gt/0019.png} \\

            \raisebox{23pt}{\rotatebox[origin=c]{90}{Cabinet}}&
            \includegraphics[width=0.18\textwidth]{./img/render_result/gs/0073.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/sugar/0073.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/pgsr/0073.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/pgsr_dn/0073.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/gt/0073.png} \\

            \raisebox{23pt}{\rotatebox[origin=c]{90}{Desk}}&
            \includegraphics[width=0.18\textwidth]{./img/render_result/gs/0024.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/sugar/0024.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/pgsr/0024.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/pgsr_dn/0024.png} &
            \includegraphics[width=0.18\textwidth]{./img/render_result/gt/0024.png} \\
        \end{tabular}
    }
    \vspace{-5pt}
    \caption{The reconstruction performance of GauStudio, SuGaR, PGSR, and our proposed method on real-world captured images is evaluated. Compared to 3DGS and SuGaR, PGSR provides an improved visual experience. Building upon PGSR, our method incorporates regularization loss terms for depth and normal vectors, achieving smoother planar surfaces, such as walls, doors, and screens, and demonstrating more robust handling of transparent surfaces like glass.}
    \label{fig:compare_of_reconstruct}
\end{figure*}
```

## Usage

Please refers to README at [post-process](https://github.com/pzhren/InfiniteWorld/blob/master/real2sim/post-process/README.md) and [depth-real2sim](https://github.com/Faccococo/PGSR?tab=readme-ov-file#pgsr-planar-based-gaussian-splatting-for-efficient-and-high-fidelity-surface-reconstruction)
