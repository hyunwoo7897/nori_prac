#include <Eigen/Geometry>
#include <bbox.h>


class Node{
    private:
    Matrix<float, 3, Dynamic> triangles;
    List<Node*> child;
    
    int maxdepth = 6;
    public :

    Node *build(bounding box, triangles, depth) {
        if (depth > maxdepth)
            return nullptr;

        //no triangles
        if (triangles)
            return nullptr;

        for (&auto triangle : triangles){
            if (box.contains(triangle))
                return nullptr;
        }
        //less triangles left
        if (length(triangles) < 10){
            LeafNode *lastNode = new LeafNode(triangles);
            return lastNode;
        }

        triangle_list list[8];
        float x_mid
        float y_
        PointType mid = (box.min + box.max) / 2; 
        for (auto& triangle : triangles) {
            TBoundingBox childBox;
            PointType boxMin; 
            PointType boxMax; 
            for (int i = 0; i < 8; ++i) {

                if(i==0){
                    childBox.min = box.min;
                    childBox.max = mid;
                }
                if(i==1){
                    childBox.min = (mid.min[0],box.min[1],box.min[2])
                    childBox.max = (...)
                }
                if(i==2)
                if(i==3)
                if(i==4)
                if(i==5)
                if(i==6)
                if(i==7)

                if (childBox.contains(triangle))
                    list[i].append(triangle);
            }
        }

        Node *node = new Node();
        for (int i = 0; i < 8; ++i)
            node.child[i] = build(bounding box of sub-node i, list[i]);
        return node;
    }

}









//Octree Implementation




class Octree{
    public:
    private:


}
