#include "MinHeap.h"

MinHeap::MinHeap(const Node& node)
{
	_vector.push_back(node);
	Heapify();
}

void MinHeap::Heapify()
{
	int length = _vector.size();
	for (int i = length - 1; i >= 0; --i)
	{
		BubbleDown(i);
	}
}

void MinHeap::BubbleDown(const int index)
{
	int length = _vector.size();
	int leftChildIndex = 2 * index + 1;
	int rightChildIndex = 2 * index + 2;

	if (leftChildIndex >= length)
		return;

	int minIndex = index;

	if (_vector.at(index).total_cost_ > _vector.at(leftChildIndex).total_cost_)
	{
		minIndex = leftChildIndex;
	}

	if ((rightChildIndex < length) && (_vector.at(minIndex).total_cost_ > _vector.at(rightChildIndex).total_cost_))
	{
		minIndex = rightChildIndex;
	}

	if (minIndex != index)
	{
		std::swap(_vector[minIndex], _vector[index]);
		BubbleDown(minIndex);
	}
}

void MinHeap::BubbleUp(const int index)
{
	if (index == 0)
		return;

	int parentIndex = (index - 1) / 2;

	if (_vector.at(parentIndex).total_cost_ > _vector.at(index).total_cost_)
	{
		std::swap(_vector[parentIndex], _vector[index]);
		BubbleUp(parentIndex);
	}
}

void MinHeap::Insert(const Node& newNode)
{
	int length = _vector.size();
	_vector.push_back(newNode);

	BubbleUp(length);
}

void MinHeap::Exchange(const Node& newNode, const int s0_)
{
        int length1 = _vector.size();
        _vector.push_back(newNode);
        for (size_t i = 0;i<length1;i++)
        {
           if ((int((_vector[length1].s_ - s0_) * 2) == int((_vector[i].s_ - s0_) * 2)) and (int(_vector[length1].v_*10) == int(_vector[i].v_*10)) and (_vector[length1].layer_ == _vector[i].layer_))
           {
                 if (_vector[length1].total_cost_ < _vector[i].total_cost_)
		{
		     std::swap(_vector[length1],_vector[i]);
                     _vector.pop_back();
                     break;
		}
                 else
                {
                     _vector.pop_back();
                     break;
                }
           }
        }
}
        

Node MinHeap::GetMin()
{
	return _vector.at(0);
}

void MinHeap::DeleteMin()
{
	int length = _vector.size();

	if (length == 0)
	{
		return;
	}
	_vector.insert(_vector.begin(), _vector.at(length - 1));
	_vector.erase(_vector.begin()+1);
	_vector.pop_back();

	BubbleDown(0);
}

int MinHeap::GetSize()
{
	return _vector.size();
}
