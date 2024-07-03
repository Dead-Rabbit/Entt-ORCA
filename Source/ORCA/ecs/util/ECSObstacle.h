#pragma once

namespace ecs {
	class ECSObstacle {
	public:
		ECSObstacle();

		bool isConvex_;
		ECSObstacle *nextObstacle_;
		FVector point_;
		ECSObstacle *prevObstacle_;
		FVector unitDir_;

		size_t id_;
	};
}