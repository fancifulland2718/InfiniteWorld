# sphinx_gallery_thumbnail_number = 1
import numpy as np
from pymeshfix import MeshFix
from pymeshfix._meshfix import PyTMesh
# from pymeshfix.examples import planar_mesh
import pyvista as pv
import argparse

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=str, required=True)
    parser.add_argument("--nbe", type=int, default=2000)
    return parser.parse_args()

def main(args):
    ##########################  加载mesh并展示其空洞 #######################################
    #读取mesh
    # print(planar_mesh)
    # planar_mesh="hole.ply"
    orig_mesh = pv.read(args.input)
    #orig_mesh = pv.read("hole.obj")
    # orig_mesh.plot_boundaries()

    save_path = args.input.replace(".ply","_r.ply")

    #计算mesh的孔洞
    meshfix = MeshFix(orig_mesh)
    holes = meshfix.extract_holes()

    #将mesh与孔洞进行叠加展示
    plotter = pv.Plotter()
    plotter.add_mesh(orig_mesh, color=True)
    plotter.add_mesh(holes, color="r", line_width=5)
    plotter.enable_eye_dome_lighting()  # helps depth perception
    orig_mesh.save(save_path)
    # _ = plotter.show()

    ############################  mesh空洞修复   ####################################
    #构建PyTMesh修复对象mfix，并加载要修复的mesh
    mfix = PyTMesh(False)  
    mfix.load_file(save_path)
    # print("aaa")

    #mfix = MeshFix(orig_mesh)
    #填充最多具有“nbe”边界边缘的所有孔,如果“refine”为true，添加内部顶点以重现采样周围环境的密度。返回修补的孔数。
    #“nbe”为0（默认值），则修复所有孔，值越大则表明
    mfix.fill_small_boundaries(nbe=args.nbe, refine=True)

    ############################  将PyTMesh对象转换为pyvista mesh #########################################
    #
    vert, faces = mfix.return_arrays()
    triangles = np.empty((faces.shape[0], 4), dtype=faces.dtype)
    triangles[:, -3:] = faces
    triangles[:, 0] = 3
    mesh = pv.PolyData(vert, triangles)

    ############################ 进行可视化，并保存mesh #########################################

    plotter = pv.Plotter()
    plotter.add_mesh(mesh, color=True)
    plotter.add_mesh(holes, color="r", line_width=5)
    plotter.enable_eye_dome_lighting()  # helps depth perception
    # _ = plotter.show()

    mesh.save(args.input.replace(".ply","_r.ply"))
    print("done")

if __name__ == "__main__":
    main(get_args())