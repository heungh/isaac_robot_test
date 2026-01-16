#!/bin/bash
# ============================================================
# Deploy Spot Robot AI Debug Infrastructure
# ============================================================

set -e

# Configuration
STACK_NAME="spot-robot-debug"
REGION="${AWS_REGION:-ap-northeast-2}"
TEMPLATE_FILE="cloudformation.yaml"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}=== Spot Robot AI Debug System Deployment ===${NC}"
echo ""

# Check AWS CLI
if ! command -v aws &> /dev/null; then
    echo -e "${RED}ERROR: AWS CLI not found. Please install it first.${NC}"
    exit 1
fi

# Check AWS credentials
if ! aws sts get-caller-identity &> /dev/null; then
    echo -e "${RED}ERROR: AWS credentials not configured.${NC}"
    echo "Run: aws configure"
    exit 1
fi

echo -e "${YELLOW}[1/4] Validating CloudFormation template...${NC}"
aws cloudformation validate-template \
    --template-body file://${TEMPLATE_FILE} \
    --region ${REGION} > /dev/null

echo -e "${GREEN}Template validation successful!${NC}"
echo ""

# Check if stack exists
STACK_EXISTS=$(aws cloudformation describe-stacks \
    --stack-name ${STACK_NAME} \
    --region ${REGION} 2>&1 || true)

if echo "$STACK_EXISTS" | grep -q "does not exist"; then
    echo -e "${YELLOW}[2/4] Creating new stack: ${STACK_NAME}...${NC}"
    aws cloudformation create-stack \
        --stack-name ${STACK_NAME} \
        --template-body file://${TEMPLATE_FILE} \
        --capabilities CAPABILITY_NAMED_IAM \
        --region ${REGION} \
        --parameters \
            ParameterKey=ProjectName,ParameterValue=${STACK_NAME} \
            ParameterKey=Environment,ParameterValue=dev

    echo -e "${YELLOW}[3/4] Waiting for stack creation...${NC}"
    aws cloudformation wait stack-create-complete \
        --stack-name ${STACK_NAME} \
        --region ${REGION}
else
    echo -e "${YELLOW}[2/4] Updating existing stack: ${STACK_NAME}...${NC}"
    aws cloudformation update-stack \
        --stack-name ${STACK_NAME} \
        --template-body file://${TEMPLATE_FILE} \
        --capabilities CAPABILITY_NAMED_IAM \
        --region ${REGION} \
        --parameters \
            ParameterKey=ProjectName,ParameterValue=${STACK_NAME} \
            ParameterKey=Environment,ParameterValue=dev \
        2>&1 || {
            if echo "$?" | grep -q "No updates"; then
                echo -e "${GREEN}No updates needed.${NC}"
            else
                exit 1
            fi
        }

    echo -e "${YELLOW}[3/4] Waiting for stack update...${NC}"
    aws cloudformation wait stack-update-complete \
        --stack-name ${STACK_NAME} \
        --region ${REGION} 2>/dev/null || true
fi

echo -e "${GREEN}Stack deployment complete!${NC}"
echo ""

# Get outputs
echo -e "${YELLOW}[4/4] Getting stack outputs...${NC}"
echo ""

OUTPUTS=$(aws cloudformation describe-stacks \
    --stack-name ${STACK_NAME} \
    --region ${REGION} \
    --query 'Stacks[0].Outputs' \
    --output json)

# Parse and display outputs
echo -e "${GREEN}=== Stack Outputs ===${NC}"
echo "$OUTPUTS" | python3 -c "
import json, sys
data = json.load(sys.stdin)
for item in data:
    print(f\"  {item['OutputKey']}: {item['OutputValue']}\")
"

# Generate config file
echo ""
echo -e "${YELLOW}Generating config file...${NC}"

BUCKET_NAME=$(echo "$OUTPUTS" | python3 -c "import json,sys; data=json.load(sys.stdin); print([o['OutputValue'] for o in data if o['OutputKey']=='DataBucketName'][0])")
FIREHOSE_NAME=$(echo "$OUTPUTS" | python3 -c "import json,sys; data=json.load(sys.stdin); print([o['OutputValue'] for o in data if o['OutputKey']=='FirehoseStreamName'][0])")

DYNAMODB_TABLE=$(echo "$OUTPUTS" | python3 -c "import json,sys; data=json.load(sys.stdin); print([o['OutputValue'] for o in data if o['OutputKey']=='ParameterHistoryTableName'][0])" 2>/dev/null || echo "spot-robot-debug-parameter-history")

cat > config.env << EOF
# Spot Robot AI Debug System Configuration
# Generated: $(date)

export AWS_REGION=${REGION}
export S3_BUCKET=${BUCKET_NAME}
export FIREHOSE_STREAM=${FIREHOSE_NAME}
export DYNAMODB_TABLE=${DYNAMODB_TABLE}
export GLUE_DATABASE=spot-robot-debug_db
export ATHENA_WORKGROUP=spot-robot-debug-workgroup
EOF

echo -e "${GREEN}Config file generated: config.env${NC}"
echo ""
echo -e "${GREEN}=== Deployment Complete ===${NC}"
echo ""
echo "Next steps:"
echo "  1. Copy config.env to EC2 instance"
echo "  2. Attach EC2 instance profile: spot-robot-debug-ec2-profile"
echo "  3. (Optional) Set Twelve Labs API key in Secrets Manager"
echo ""
