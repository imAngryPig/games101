#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    // printf("%d\n\n",primitives.size());
    if(splitMethod == SplitMethod::NAIVE)
    root = recursiveBuild(primitives);
    else if(splitMethod == SplitMethod::SAH)
    root = recursiveSAH(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());  //合并整个objects成一个大boundingbox

    if (objects.size() == 1) {    // 只有一个object
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        //找到每个三角形包围盒的中心点的大包围盒
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());

        // dim 表示 整个mesh里面的三角形的包围盒的中心点形成的大包围盒的从左下到右上点最大跨度的轴
        // 0 : x  ;  1 : y ;  2 : z
        int dim = centroidBounds.maxExtent();
        // 按跨度最大的轴来进行包围盒中心点排序
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        // 寻找处于按照最大轴排序后，处于中间位置的object
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        // 左边一半的object集合
        auto leftshapes = std::vector<Object*>(beginning, middling);
        // 右边一半的object集合
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

// SAH加速方法
BVHBuildNode* BVHAccel::recursiveSAH(std::vector<Object*> objects){
    BVHBuildNode* node = new BVHBuildNode();

    Bounds3 bounds;
    for(int i=0;i<objects.size();++i)
        bounds = Union(bounds,objects[i]->getBounds());
    if(objects.size()==1){
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if(objects.size()==2){
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else    // 主要分割阶段
    {
        auto beginning = objects.begin();
        auto ending = objects.end();

        if(objects.size()<12){  //物体数小于12就开摆，中间割
            auto middling = objects.begin() + (objects.size() / 2);
            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);
            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);
            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        else  // 物体数量大于12时正式开始SAH方法
        {
            int bestChoice=0;
            double minCost = std::numeric_limits<double>::max();
            int bestDim=0;
            for(int dim=0;dim<3;++dim){
                switch (dim) {
                    case 0:
                        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                            return f1->getBounds().Centroid().x <
                                f2->getBounds().Centroid().x;
                        });
                        break;
                    case 1:
                        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                            return f1->getBounds().Centroid().y <
                                f2->getBounds().Centroid().y;
                        });
                        break;
                    case 2:
                        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                            return f1->getBounds().Centroid().z <
                                f2->getBounds().Centroid().z;
                        });
                        break;
                    }

                    float l = (float)objects.size();
                    float nums[] = { 1.0/6 , 2.0/6 , 3.0/6 , 4.0/6 , 5.0/6};
                    for(int i=0;i<5;i++){
                        nums[i]*=l;        // 按照1/6, 2/6 ,3/6 ,4/6 ,5/6这五个界限来枚举最小cost
                    }
                    // cost(A,B)=t_traversal + P_A*∑(ai=1~number of ABoundingbox)t_intersect(ai) 
                    //  + P_B*∑(bi=1~number of ABoundingbox)t_intersect(bi)
                    // 在具体实现中，我们假设t_traversal=1=t_intersect(ai)
                    // 同时，我们取P_A,P_B为A,B各自包围体体积所占大的包围体的体积的比例
                    for(int i=0;i<5;i++){
                        auto middling = objects.begin() + (int)nums[i];
                        auto leftshapes = std::vector<Object*>(beginning, middling);
                        auto rightshapes = std::vector<Object*>(middling, ending);
                        Bounds3 leftBound,rightBound;
                        for(int i=0;i<leftshapes.size();i++) Union(leftBound,leftshapes[i]->getBounds());
                        for(int i=0;i<rightshapes.size();i++) Union(rightBound,rightshapes[i]->getBounds());
                        auto leftBoxSize=leftBound.BoxVolunm(); auto rightBoxSize=rightBound.BoxVolunm();
                        double cost = 5.0f + leftBoxSize/(leftBoxSize+rightBoxSize) * leftshapes.size() +
                        rightBoxSize/(leftBoxSize+rightBoxSize) * rightshapes.size();
                        if(cost < minCost){
                            minCost = cost;
                            bestChoice = (int)nums[i];
                            bestDim = dim;
                        }
                    } //end for i=1~5
            }// end for 
            switch(bestDim)
            { // 不对Z排序的原因是在上述for循环中最后一次就是对z轴排序完成了
                case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
                });
                break;
                case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
                });
                break;
            }//end switch
            auto middling =objects.begin() + bestChoice;
            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);
            node->left = recursiveSAH(leftshapes);
            node->right = recursiveSAH(rightshapes);
            node->bounds = Union(node->left->bounds,node->right->bounds);
        }// end object>=12 
    }// end SAH accelerating
    return node;
}


Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection inter;
    if(node==nullptr) return inter;
    std::array<int,3> dirIsNeg;
    dirIsNeg[0] = ray.direction.x<0;
    dirIsNeg[1] = ray.direction.y<0; 
    dirIsNeg[2] = ray.direction.z<0;

    if(!node->bounds.IntersectP(ray,ray.direction_inv,dirIsNeg))
        return inter;
    else
    {
        if(node->left==nullptr && node->right==nullptr)
            return node->object->getIntersection(ray);
    }

    auto hit1=getIntersection(node->left,ray);
    auto hit2=getIntersection(node->right,ray);

    return hit1.distance<hit2.distance?hit1:hit2;

}