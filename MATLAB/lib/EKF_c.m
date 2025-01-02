classdef EKF_c
    properties
        x % State Vector
        z % Measurement Vector
        P % Covariance Matrix
        Q % Process Noise Covariance Matrix
        R % Sensor Noise Covariance Matrix
    end

    methods
        function obj = EKF_c(initialState, initialCovariance, processNoise, measurementNoise)
            obj.x = initialState;
            obj.P = initialCovariance;
            obj.Q = processNoise;
            obj.R = measurementNoise;

            % 상태 벡터 크기 확인
            stateSize = size(initialState, 1);
            measurementSize = size(measurementNoise, 1);

            obj.z = zeros(measurementSize, 1);


            % 예외 처리: Q의 크기 확인
            if ~isequal(size(processNoise), [stateSize, stateSize])
                error('Process noise Q must be a square matrix with size [%d, %d].', stateSize, stateSize);
            end

            % 예외 처리: P의 크기 확인
            if ~isequal(size(initialCovariance), [stateSize, stateSize])
                error('Initial covariance P must be a square matrix with size [%d, %d].', stateSize, stateSize);
            end
        end


        function state = stateTransitionFcn(obj, state, controlInput)
            % 상태 전이 로직
            state = state + controlInput;
        end

        function measurement = measurementFcn(obj, state)
            % 측정 함수 로직
            measurement = state;
        end

        function obj = predict(obj, controlInput)
            % 상태 전이
            obj.x = obj.stateTransitionFcn(obj.x, controlInput);

            % 공분산 갱신 (Jacobian 생략)
            obj.P = obj.P + obj.Q;
        end

        function obj = update(obj, measurement)
            % 예측 측정값
            predictedMeasurement = obj.measurementFcn(obj.x);

            % 측정 업데이트 로직 (칼만 이득 생략)
            obj.x = obj.x + (measurement - predictedMeasurement);
        end
    end
end
